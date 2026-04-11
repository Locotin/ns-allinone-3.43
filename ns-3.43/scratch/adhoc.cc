#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HierarchicalPrecisionAgriManet");

namespace
{

// -----------------------------------------------------------------------------
// Este archivo implementa un experimento MANET jerárquico con dos niveles:
// 1. Agrobots: recolectan datos desde sensores estáticos del campo.
// 2. UGVs: actúan como segundo relevo móvil y entregan a la casa.
//
// La lógica de red usa store-carry-forward real por muestra, con buffers, TTL
// y contactos entre sensores, agrobots, UGVs y casa.
//
// La intención de estos comentarios es explicar:
// - el rol metodológico de cada bloque;
// - por qué se eligieron ciertas APIs de ns-3;
// - cómo se traducen los conceptos del experimento a objetos del simulador.
// -----------------------------------------------------------------------------

// StageMetrics resume lo que FlowMonitor reporta por cada salto lógico.
// No modela SCF por sí mismo; solo sintetiza TX/RX y delay IP/UDP por etapa.
struct StageMetrics
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t rxDelaySamples{0};
    double totalDelaySeconds{0.0};
};

// BufferedSample representa una muestra almacenada localmente en un nodo SCF.
// Guardamos el paquete y metadatos suficientes para medir TTL, carry time
// y deduplicación por sensor/sampleId.
struct BufferedSample
{
    Ptr<Packet> packet;
    uint64_t key{0};
    Time originTime{Seconds(0)};
    uint32_t sensorId{0};
    uint32_t sampleId{0};
};

// LayerAggregateStats agrega solo las estadísticas de capa que hoy alimentan
// los dashboards: drops por TTL y ocupación media de buffer.
struct LayerAggregateStats
{
    uint32_t nodeCount{0};
    uint64_t ttlDrops{0};
    double averageOccupancySum{0.0};
};

// SampleHeader es un encabezado de aplicación propio.
// Se eligió heredar de ns3::Header porque la documentación de ns-3 recomienda
// serializar metadatos de protocolo/experimento como headers explícitos cuando
// deben viajar dentro del Packet y recuperarse en nodos intermedios.
//
// Aquí el header preserva identidad de la muestra y su tiempo de origen a través
// de todos los relevos, algo indispensable para delay extremo a extremo y AoI.
class SampleHeader : public Header
{
  public:
    static constexpr uint32_t SERIALIZED_SIZE = 17;

    SampleHeader();

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    void Print(std::ostream& os) const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

    void SetSensorId(uint32_t sensorId);
    void SetSampleId(uint32_t sampleId);
    void SetOriginTimestamp(Time timestamp);
    void SetLogicalHop(uint8_t logicalHop);

    uint32_t GetSensorId() const;
    uint32_t GetSampleId() const;
    Time GetOriginTimestamp() const;
  private:
    uint32_t m_sensorId{0};
    uint32_t m_sampleId{0};
    uint64_t m_originTimestampNs{0};
    uint8_t m_logicalHop{0};
};

NS_OBJECT_ENSURE_REGISTERED(SampleHeader);

SampleHeader::SampleHeader() = default;

TypeId
SampleHeader::GetTypeId()
{
    // GetTypeId registra la clase en el sistema de tipos de ns-3.
    // Esto permite introspección, creación dinámica y trazabilidad.
    static TypeId tid = TypeId("SampleHeader").SetParent<Header>().AddConstructor<SampleHeader>();
    return tid;
}

TypeId
SampleHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
SampleHeader::Print(std::ostream& os) const
{
    // Print se usa cuando el paquete/header se inspecciona en logs o depuración.
    os << "sensor=" << m_sensorId << ", sample=" << m_sampleId
       << ", originNs=" << m_originTimestampNs << ", hop=" << static_cast<uint32_t>(m_logicalHop);
}

uint32_t
SampleHeader::GetSerializedSize() const
{
    return SERIALIZED_SIZE;
}

void
SampleHeader::Serialize(Buffer::Iterator start) const
{
    // Usamos enteros en orden de red para que el contenido del header viaje de
    // forma determinística dentro del Packet y pueda reconstruirse exactamente.
    start.WriteHtonU32(m_sensorId);
    start.WriteHtonU32(m_sampleId);
    start.WriteHtonU64(m_originTimestampNs);
    start.WriteU8(m_logicalHop);
}

uint32_t
SampleHeader::Deserialize(Buffer::Iterator start)
{
    // Deserialize invierte la serialización y recupera el estado del header
    // cuando un nodo intermedio o la casa recibe una muestra.
    m_sensorId = start.ReadNtohU32();
    m_sampleId = start.ReadNtohU32();
    m_originTimestampNs = start.ReadNtohU64();
    m_logicalHop = start.ReadU8();
    return SERIALIZED_SIZE;
}

void
SampleHeader::SetSensorId(uint32_t sensorId)
{
    m_sensorId = sensorId;
}

void
SampleHeader::SetSampleId(uint32_t sampleId)
{
    m_sampleId = sampleId;
}

void
SampleHeader::SetOriginTimestamp(Time timestamp)
{
    m_originTimestampNs = static_cast<uint64_t>(timestamp.GetNanoSeconds());
}

void
SampleHeader::SetLogicalHop(uint8_t logicalHop)
{
    m_logicalHop = logicalHop;
}

uint32_t
SampleHeader::GetSensorId() const
{
    return m_sensorId;
}

uint32_t
SampleHeader::GetSampleId() const
{
    return m_sampleId;
}

Time
SampleHeader::GetOriginTimestamp() const
{
    return NanoSeconds(m_originTimestampNs);
}

class StoreCarryForwardApp : public Application
{
  public:
    static TypeId GetTypeId();

    StoreCarryForwardApp();

    void SetRoleName(const std::string& roleName);
    void SetListenPort(uint16_t listenPort);
    void SetNextHop(Ptr<Node> nextHopNode, Ipv4Address nextHopAddress, uint16_t nextHopPort);
    void SetStoreCarryForwardParameters(uint32_t bufferCapacityPackets,
                                        Time sampleTtl,
                                        Time contactCheckInterval,
                                        uint32_t forwardBurstPackets,
                                        double contactRange);
    void SetForwardHop(uint8_t logicalHop);
    void SetSourceParameters(uint32_t sourceId,
                             Time generationInterval,
                             uint32_t packetSize,
                             Time initialGenerationDelay);

    uint64_t GetTtlDrops() const;
    double GetAverageOccupancy() const;
    uint32_t GetUniqueSourceCount() const;

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;
    void GenerateSample();
    void CheckContactAndForward();
    void HandleRead(Ptr<Socket> socket);
    bool EnqueueSample(Ptr<Packet> packet, const SampleHeader& header, bool countAsReceived);
    void PurgeExpiredSamples();
    bool IsInContact() const;
    void UpdateOccupancyIntegral();
    uint64_t MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const;

    std::string m_roleName{"relay"};
    bool m_running{false};
    bool m_isSource{false};
    uint32_t m_sourceId{0};
    uint32_t m_nextSampleId{0};
    uint32_t m_packetSize{SampleHeader::SERIALIZED_SIZE};
    uint16_t m_listenPort{0};
    uint16_t m_nextHopPort{0};
    uint8_t m_logicalHop{0};
    uint32_t m_bufferCapacityPackets{128};
    uint32_t m_forwardBurstPackets{4};
    double m_contactRange{30.0};
    Time m_generationInterval{Seconds(3.0)};
    Time m_initialGenerationDelay{Seconds(0.0)};
    Time m_sampleTtl{Seconds(120.0)};
    Time m_contactCheckInterval{Seconds(1.0)};
    Time m_activeStart{Seconds(0.0)};
    Time m_lastOccupancyUpdate{Seconds(0.0)};
    Time m_activeDuration{Seconds(0.0)};
    double m_occupancyIntegral{0.0};
    uint64_t m_ttlDrops{0};
    EventId m_generationEvent;
    EventId m_contactEvent;
    Ptr<Node> m_nextHopNode;
    Ipv4Address m_nextHopAddress;
    Ptr<Socket> m_rxSocket;
    Ptr<Socket> m_txSocket;
    std::deque<BufferedSample> m_buffer;
    std::unordered_set<uint64_t> m_seenSampleKeys;
	    std::unordered_set<uint32_t> m_uniqueSourceIds;
	};

NS_OBJECT_ENSURE_REGISTERED(StoreCarryForwardApp);

TypeId
StoreCarryForwardApp::GetTypeId()
{
    // La app se registra como ns3::Application para poder instalarse en nodos
    // sin modificar módulos base del simulador, que es la vía correcta para un
    // experimento en scratch/.
    static TypeId tid =
        TypeId("StoreCarryForwardApp").SetParent<Application>().AddConstructor<StoreCarryForwardApp>();
    return tid;
}

StoreCarryForwardApp::StoreCarryForwardApp() = default;

void
StoreCarryForwardApp::SetRoleName(const std::string& roleName)
{
    m_roleName = roleName;
}

void
StoreCarryForwardApp::SetListenPort(uint16_t listenPort)
{
    m_listenPort = listenPort;
}

void
StoreCarryForwardApp::SetNextHop(Ptr<Node> nextHopNode, Ipv4Address nextHopAddress, uint16_t nextHopPort)
{
    m_nextHopNode = nextHopNode;
    m_nextHopAddress = nextHopAddress;
    m_nextHopPort = nextHopPort;
}

void
StoreCarryForwardApp::SetStoreCarryForwardParameters(uint32_t bufferCapacityPackets,
                                                     Time sampleTtl,
                                                     Time contactCheckInterval,
                                                     uint32_t forwardBurstPackets,
                                                     double contactRange)
{
    // Este bloque concentra los parámetros metodológicos del patrón SCF:
    // cuánto cabe en buffer, cuánto dura una muestra, cada cuánto se revisa
    // contacto y cuántas muestras se vacían por encuentro.
    m_bufferCapacityPackets = bufferCapacityPackets;
    m_sampleTtl = sampleTtl;
    m_contactCheckInterval = contactCheckInterval;
    m_forwardBurstPackets = std::max<uint32_t>(1, forwardBurstPackets);
    m_contactRange = contactRange;
}

void
StoreCarryForwardApp::SetForwardHop(uint8_t logicalHop)
{
    m_logicalHop = logicalHop;
}

void
StoreCarryForwardApp::SetSourceParameters(uint32_t sourceId,
                                          Time generationInterval,
                                          uint32_t packetSize,
                                          Time initialGenerationDelay)
{
    // Solo los sensores llaman a esta función. Los relays agrobot/UGV no
    // generan muestras nuevas; solo almacenan y reenvían las que reciben.
    m_isSource = true;
    m_sourceId = sourceId;
    m_generationInterval = generationInterval;
    m_packetSize = packetSize;
    m_initialGenerationDelay = initialGenerationDelay;
}

uint64_t
StoreCarryForwardApp::GetTtlDrops() const
{
    return m_ttlDrops;
}

double
StoreCarryForwardApp::GetAverageOccupancy() const
{
    if (m_activeDuration.IsZero())
    {
        return 0.0;
    }

    return m_occupancyIntegral / m_activeDuration.GetSeconds();
}

uint32_t
StoreCarryForwardApp::GetUniqueSourceCount() const
{
    return m_uniqueSourceIds.size();
}

void
StoreCarryForwardApp::DoDispose()
{
    // DoDispose libera referencias ns-3/Ptr y limpia estructuras auxiliares.
    // Esto evita que queden sockets o punteros vivos tras Simulator::Destroy().
    m_buffer.clear();
    m_seenSampleKeys.clear();
    m_uniqueSourceIds.clear();
    m_nextHopNode = nullptr;
    m_rxSocket = nullptr;
    m_txSocket = nullptr;
    Application::DoDispose();
}

void
StoreCarryForwardApp::StartApplication()
{
    // StartApplication es el punto de entrada estándar de ns3::Application.
    // Aquí la app abre sockets, programa generación periódica y activa el lazo
    // de chequeo de contactos. La documentación de ns-3 recomienda esta lógica
    // dentro del ciclo Start/Stop y no en el constructor.
    m_running = true;
    m_activeStart = Simulator::Now();
    m_lastOccupancyUpdate = Simulator::Now();

    if (m_listenPort != 0 && !m_rxSocket)
    {
        // UdpSocketFactory es la fábrica estándar de sockets UDP en ns-3.
        // La usamos porque el experimento modela intercambio simple de datagramas
        // y evita agregar complejidad de transporte confiable.
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_rxSocket = Socket::CreateSocket(GetNode(), tid);
        if (m_rxSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_listenPort)) == -1)
        {
            NS_FATAL_ERROR("No se pudo hacer bind del socket UDP para " << m_roleName);
        }
        m_rxSocket->SetRecvCallback(MakeCallback(&StoreCarryForwardApp::HandleRead, this));
    }

    if (m_nextHopPort != 0 && !m_txSocket)
    {
        // El socket emisor se conecta una sola vez al siguiente salto lógico.
        // En SCF el "si puedo o no puedo enviar ahora" no depende del socket
        // sino del criterio de contacto por distancia que aplica la aplicación.
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_txSocket = Socket::CreateSocket(GetNode(), tid);
        if (m_txSocket->Bind() == -1)
        {
            NS_FATAL_ERROR("No se pudo hacer bind del socket emisor UDP para " << m_roleName);
        }
        if (m_txSocket->Connect(InetSocketAddress(m_nextHopAddress, m_nextHopPort)) == -1)
        {
            NS_FATAL_ERROR("No se pudo conectar el socket emisor UDP para " << m_roleName);
        }
    }

    if (m_isSource)
    {
        m_generationEvent =
            Simulator::Schedule(m_initialGenerationDelay, &StoreCarryForwardApp::GenerateSample, this);
    }

    if (m_nextHopNode && m_nextHopPort != 0)
    {
        m_contactEvent =
            Simulator::Schedule(Seconds(0.1), &StoreCarryForwardApp::CheckContactAndForward, this);
    }
}

void
StoreCarryForwardApp::StopApplication()
{
    m_running = false;
    if (m_generationEvent.IsPending())
    {
        Simulator::Cancel(m_generationEvent);
    }
    if (m_contactEvent.IsPending())
    {
        Simulator::Cancel(m_contactEvent);
    }

    UpdateOccupancyIntegral();
    m_activeDuration = Simulator::Now() - m_activeStart;

    if (m_rxSocket)
    {
        m_rxSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        m_rxSocket->Close();
    }
    if (m_txSocket)
    {
        m_txSocket->Close();
    }
}

void
StoreCarryForwardApp::GenerateSample()
{
    // Los sensores generan una muestra, la encapsulan con SampleHeader y la
    // encolan localmente. Aunque exista conectividad, la app sigue la misma
    // ruta SCF: primero guarda, luego un chequeo de contacto la reenvía.
    if (!m_running)
    {
        return;
    }

    PurgeExpiredSamples();

    SampleHeader header;
    // El header identifica unívocamente la medición y fija su timestamp origen.
    header.SetSensorId(m_sourceId);
    header.SetSampleId(m_nextSampleId++);
    header.SetOriginTimestamp(Simulator::Now());
    header.SetLogicalHop(m_logicalHop);

    const uint32_t payloadSize =
        (m_packetSize > SampleHeader::SERIALIZED_SIZE) ? (m_packetSize - SampleHeader::SERIALIZED_SIZE)
                                                       : 0;
    // Creamos payload "vacío" del tamaño requerido y luego añadimos el header.
    // En este experimento importa más el transporte de metadatos que el contenido.
    Ptr<Packet> packet = Create<Packet>(payloadSize);
    packet->AddHeader(header);

    EnqueueSample(packet, header, false);

    m_generationEvent =
        Simulator::Schedule(m_generationInterval, &StoreCarryForwardApp::GenerateSample, this);
}

void
StoreCarryForwardApp::CheckContactAndForward()
{
    // Este es el corazón de store-carry-forward:
    // 1. purga expiradas,
    // 2. verifica si el siguiente salto está dentro del rango de contacto,
    // 3. reenvía hasta un máximo de burst por oportunidad.
    if (!m_running)
    {
        return;
    }

    PurgeExpiredSamples();

    if (!m_buffer.empty() && m_nextHopNode && m_txSocket && IsInContact())
    {
        uint32_t forwardedThisRound = 0;
        while (forwardedThisRound < m_forwardBurstPackets && !m_buffer.empty())
        {
            const BufferedSample record = m_buffer.front();
            SampleHeader header;
            Ptr<Packet> inspect = record.packet->Copy();
            inspect->RemoveHeader(header);
            // Reescribimos el logicalHop para reflejar desde qué capa sale ahora
            // la muestra, pero preservamos sensorId/sampleId/originTimestamp.
            header.SetLogicalHop(m_logicalHop);
            const uint32_t payloadSize =
                (record.packet->GetSize() > SampleHeader::SERIALIZED_SIZE)
                    ? (record.packet->GetSize() - SampleHeader::SERIALIZED_SIZE)
                    : 0;
            Ptr<Packet> outbound = Create<Packet>(payloadSize);
            outbound->AddHeader(header);

            if (m_txSocket->Send(outbound) < 0)
            {
                // Si el socket no acepta el envío, se conserva el resto del buffer
                // para próximos contactos; no se destruye el estado SCF.
                break;
            }

            UpdateOccupancyIntegral();
            m_buffer.pop_front();

            ++forwardedThisRound;
        }
    }

    m_contactEvent = Simulator::Schedule(m_contactCheckInterval,
                                         &StoreCarryForwardApp::CheckContactAndForward,
                                         this);
}

void
StoreCarryForwardApp::HandleRead(Ptr<Socket> socket)
{
    // HandleRead recibe datagramas UDP y los mete al buffer local del relay.
    // La app no reenvía "en caliente"; primero normaliza todo a la cola SCF.
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        Ptr<Packet> inspect = packet->Copy();
        SampleHeader header;
        inspect->RemoveHeader(header);
        EnqueueSample(packet, header, true);
    }
}

bool
StoreCarryForwardApp::EnqueueSample(Ptr<Packet> packet, const SampleHeader& header, bool countAsReceived)
{
    // Cada muestra se indexa por (sensorId, sampleId). Esa clave evita contar
    // duplicados múltiples veces en relays y mantiene trazabilidad por muestra.
    const uint64_t key = MakeSampleKey(header.GetSensorId(), header.GetSampleId());

    if (countAsReceived)
    {
        if (m_seenSampleKeys.find(key) != m_seenSampleKeys.end())
        {
            return false;
        }
        m_seenSampleKeys.insert(key);
        m_uniqueSourceIds.insert(header.GetSensorId());
    }

    if (m_buffer.size() >= m_bufferCapacityPackets)
    {
        // Un overflow representa una limitación local del relay, no del canal.
        return false;
    }

    UpdateOccupancyIntegral();

    BufferedSample sample;
    sample.packet = packet->Copy();
    sample.key = key;
    sample.originTime = header.GetOriginTimestamp();
    sample.sensorId = header.GetSensorId();
    sample.sampleId = header.GetSampleId();
    m_buffer.push_back(sample);

    return true;
}

void
StoreCarryForwardApp::PurgeExpiredSamples()
{
    // TTL se evalúa contra originTime y no contra enqueueTime porque la edad
    // relevante es la de la información desde que nació en el sensor.
    auto it = m_buffer.begin();
    while (it != m_buffer.end())
    {
        if ((Simulator::Now() - it->originTime) > m_sampleTtl)
        {
            UpdateOccupancyIntegral();
            it = m_buffer.erase(it);
            ++m_ttlDrops;
        }
        else
        {
            ++it;
        }
    }
}

bool
StoreCarryForwardApp::IsInContact() const
{
    // La oportunidad de contacto se define por distancia geométrica entre nodos.
    // Esto desacopla el modelo SCF de la existencia de una ruta continua AODV.
    if (!m_nextHopNode)
    {
        return false;
    }

    Ptr<MobilityModel> selfMobility = GetNode()->GetObject<MobilityModel>();
    Ptr<MobilityModel> peerMobility = m_nextHopNode->GetObject<MobilityModel>();

    if (!selfMobility || !peerMobility)
    {
        return false;
    }

    const double distance =
        CalculateDistance(selfMobility->GetPosition(), peerMobility->GetPosition());
    return distance <= m_contactRange;
}

void
StoreCarryForwardApp::UpdateOccupancyIntegral()
{
    // Integramos ocupación en el tiempo para obtener promedio temporal real
    // del buffer, no solo snapshots puntuales.
    const double deltaSeconds = (Simulator::Now() - m_lastOccupancyUpdate).GetSeconds();
    m_occupancyIntegral += deltaSeconds * static_cast<double>(m_buffer.size());
    m_lastOccupancyUpdate = Simulator::Now();
}

uint64_t
StoreCarryForwardApp::MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const
{
    return (static_cast<uint64_t>(sensorId) << 32) | static_cast<uint64_t>(sampleId);
}

class HouseCollectorApp : public Application
{
  public:
    static TypeId GetTypeId();

    HouseCollectorApp();

    void Configure(uint16_t listenPort, uint32_t sensorCount, Time aoiSamplePeriod);

    uint32_t GetSensorsDeliveredToHouse() const;
    double GetAverageEndToEndDelay() const;
    double GetAverageAoi() const;
    double GetCurrentAverageAoi() const;

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket);
    void SampleAoi();
    void RecordAoiSnapshot();
    uint64_t MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const;

    bool m_running{false};
    uint16_t m_listenPort{0};
    uint32_t m_sensorCount{0};
    Time m_aoiSamplePeriod{Seconds(1.0)};
    Ptr<Socket> m_rxSocket;
    EventId m_aoiEvent;
    std::unordered_set<uint64_t> m_deliveredKeys;
    std::vector<bool> m_sensorDelivered;
    std::vector<bool> m_hasFreshUpdate;
    std::vector<Time> m_latestOriginTimestamp;
    double m_delaySumSeconds{0.0};
    uint64_t m_delaySamples{0};
    double m_aoiSnapshotSumSeconds{0.0};
    uint64_t m_aoiSnapshotSamples{0};
};

// La casa es el sumidero final del experimento.
// Esta app no reenvía: recibe muestras, deduplica, mide delay extremo a extremo
// y calcula Age of Information siguiendo la definición clásica de "edad actual
// de la actualización más reciente disponible".
NS_OBJECT_ENSURE_REGISTERED(HouseCollectorApp);

TypeId
HouseCollectorApp::GetTypeId()
{
    static TypeId tid =
        TypeId("HouseCollectorApp").SetParent<Application>().AddConstructor<HouseCollectorApp>();
    return tid;
}

HouseCollectorApp::HouseCollectorApp() = default;

void
HouseCollectorApp::Configure(uint16_t listenPort, uint32_t sensorCount, Time aoiSamplePeriod)
{
    // Inicializamos estructuras por sensor porque AoI y cobertura final se
    // miden a nivel de fuente original, no solo de flujos agregados.
    m_listenPort = listenPort;
    m_sensorCount = sensorCount;
    m_aoiSamplePeriod = aoiSamplePeriod;
    m_sensorDelivered.assign(sensorCount, false);
    m_hasFreshUpdate.assign(sensorCount, false);
    m_latestOriginTimestamp.assign(sensorCount, Seconds(0.0));
}

uint32_t
HouseCollectorApp::GetSensorsDeliveredToHouse() const
{
    return std::count(m_sensorDelivered.begin(), m_sensorDelivered.end(), true);
}

double
HouseCollectorApp::GetAverageEndToEndDelay() const
{
    if (m_delaySamples == 0)
    {
        return 0.0;
    }
    return m_delaySumSeconds / static_cast<double>(m_delaySamples);
}

double
HouseCollectorApp::GetAverageAoi() const
{
    if (m_aoiSnapshotSamples == 0)
    {
        return 0.0;
    }
    return m_aoiSnapshotSumSeconds / static_cast<double>(m_aoiSnapshotSamples);
}

double
HouseCollectorApp::GetCurrentAverageAoi() const
{
    double sumAoi = 0.0;
    uint32_t activeSensors = 0;

    for (uint32_t i = 0; i < m_sensorCount; ++i)
    {
        if (!m_hasFreshUpdate[i])
        {
            continue;
        }
        sumAoi += (Simulator::Now() - m_latestOriginTimestamp[i]).GetSeconds();
        ++activeSensors;
    }

    if (activeSensors == 0)
    {
        return 0.0;
    }
    return sumAoi / static_cast<double>(activeSensors);
}

void
HouseCollectorApp::DoDispose()
{
    m_rxSocket = nullptr;
    m_deliveredKeys.clear();
    m_sensorDelivered.clear();
    m_hasFreshUpdate.clear();
    m_latestOriginTimestamp.clear();
    Application::DoDispose();
}

void
HouseCollectorApp::StartApplication()
{
    // Igual que en StoreCarryForwardApp, la documentación de ns-3 favorece
    // abrir sockets en StartApplication para respetar el ciclo de vida.
    m_running = true;

    if (!m_rxSocket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_rxSocket = Socket::CreateSocket(GetNode(), tid);
        if (m_rxSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_listenPort)) == -1)
        {
            NS_FATAL_ERROR("No se pudo hacer bind del socket de la casa");
        }
        m_rxSocket->SetRecvCallback(MakeCallback(&HouseCollectorApp::HandleRead, this));
    }

    m_aoiEvent = Simulator::Schedule(m_aoiSamplePeriod, &HouseCollectorApp::SampleAoi, this);
}

void
HouseCollectorApp::StopApplication()
{
    m_running = false;
    if (m_aoiEvent.IsPending())
    {
        Simulator::Cancel(m_aoiEvent);
    }
    if (m_rxSocket)
    {
        m_rxSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        m_rxSocket->Close();
    }
}

void
HouseCollectorApp::HandleRead(Ptr<Socket> socket)
{
    // Cada muestra recibida se evalúa en cuatro dimensiones:
    // duplicidad, delay extremo a extremo, cobertura por sensor y frescura.
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        Ptr<Packet> inspect = packet->Copy();
        SampleHeader header;
        inspect->RemoveHeader(header);

        const uint64_t key = MakeSampleKey(header.GetSensorId(), header.GetSampleId());
        if (m_deliveredKeys.find(key) != m_deliveredKeys.end())
        {
            continue;
        }
        m_deliveredKeys.insert(key);

        const double delaySeconds = (Simulator::Now() - header.GetOriginTimestamp()).GetSeconds();
        // Delay e2e se calcula con el timestamp origen preservado en SampleHeader.
        m_delaySumSeconds += delaySeconds;
        ++m_delaySamples;

        if (header.GetSensorId() < m_sensorCount)
        {
            m_sensorDelivered[header.GetSensorId()] = true;
            if (!m_hasFreshUpdate[header.GetSensorId()] ||
                header.GetOriginTimestamp() > m_latestOriginTimestamp[header.GetSensorId()])
            {
                m_hasFreshUpdate[header.GetSensorId()] = true;
                m_latestOriginTimestamp[header.GetSensorId()] = header.GetOriginTimestamp();
            }
        }
    }

    RecordAoiSnapshot();
}

void
HouseCollectorApp::SampleAoi()
{
    // AoI también se muestrea periódicamente porque interesa su evolución en el
    // tiempo, no solo el valor en instantes de recepción.
    if (!m_running)
    {
        return;
    }

    RecordAoiSnapshot();
    m_aoiEvent = Simulator::Schedule(m_aoiSamplePeriod, &HouseCollectorApp::SampleAoi, this);
}

void
HouseCollectorApp::RecordAoiSnapshot()
{
    // Promediamos AoI solo sobre sensores con al menos una actualización fresca.
    // Sensores sin entregas todavía no aportan AoI "observable" en la casa.
    double sumAoi = 0.0;
    uint32_t activeSensors = 0;

    for (uint32_t i = 0; i < m_sensorCount; ++i)
    {
        if (!m_hasFreshUpdate[i])
        {
            continue;
        }

        const double ageSeconds = (Simulator::Now() - m_latestOriginTimestamp[i]).GetSeconds();
        sumAoi += ageSeconds;
        ++activeSensors;
    }

    if (activeSensors > 0)
    {
        m_aoiSnapshotSumSeconds += sumAoi / static_cast<double>(activeSensors);
        ++m_aoiSnapshotSamples;
    }
}

uint64_t
HouseCollectorApp::MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const
{
    return (static_cast<uint64_t>(sensorId) << 32) | static_cast<uint64_t>(sampleId);
}

static std::string
MakeUniformRv(double minValue, double maxValue)
{
    // ns-3 acepta variables aleatorias como StringValue en muchos helpers.
    // Esta utilidad evita repetir cadenas largas y mantiene parámetros claros.
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "ns3::UniformRandomVariable[Min=" << minValue << "|Max=" << maxValue << "]";
    return oss.str();
}

static std::string
MakeConstantRv(double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "ns3::ConstantRandomVariable[Constant=" << value << "]";
    return oss.str();
}

static Ptr<RandomRectanglePositionAllocator>
CreateUniformPositionAllocator(const Rectangle& bounds)
{
    // RandomRectanglePositionAllocator distribuye nodos uniformemente sobre un
    // rectángulo. Aquí se usa para sensores y offsets locales de clusters.
    Ptr<RandomRectanglePositionAllocator> posAlloc = CreateObject<RandomRectanglePositionAllocator>();
    posAlloc->SetAttribute("X", StringValue(MakeUniformRv(bounds.xMin, bounds.xMax)));
    posAlloc->SetAttribute("Y", StringValue(MakeUniformRv(bounds.yMin, bounds.yMax)));
    return posAlloc;
}

static void
ConfigureSweepWaypoints(const NodeContainer& leaders,
                        double simTime,
                        const Rectangle& fieldBounds,
                        double leaderSpeed,
                        bool horizontalSweep)
{
    // WaypointMobilityModel se eligió porque la documentación de ns-3 lo usa
    // para trayectorias deterministas basadas en marcas de tiempo/posición.
    // En este experimento sirve para recorridos repetibles de líderes.
    const double speed = std::max(0.5, leaderSpeed);

    if (horizontalSweep)
    {
        const double xMin = fieldBounds.xMin + 5.0;
        const double xMax = fieldBounds.xMax - 5.0;
        const double legDuration = std::max(1.0, (xMax - xMin) / speed);
        const double laneSpacing =
            (fieldBounds.yMax - fieldBounds.yMin) / static_cast<double>(leaders.GetN() + 1);

        for (uint32_t i = 0; i < leaders.GetN(); ++i)
        {
            Ptr<WaypointMobilityModel> waypoint = leaders.Get(i)->GetObject<WaypointMobilityModel>();
            if (!waypoint)
            {
                NS_FATAL_ERROR("WaypointMobilityModel no instalado en lider horizontal " << i);
            }

            const double laneY = fieldBounds.yMin + laneSpacing * static_cast<double>(i + 1);
            double t = 0.0;
            double x = xMin;

            waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
            // Cada waypoint fija una trayectoria de barrido por carriles.
            while (t + legDuration <= simTime + legDuration)
            {
                t += legDuration;
                x = (x == xMin) ? xMax : xMin;
                waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
            }
        }
        return;
    }

    const double yMin = fieldBounds.yMin + 5.0;
    const double yMax = fieldBounds.yMax - 5.0;
    const double legDuration = std::max(1.0, (yMax - yMin) / speed);
    const double laneSpacing =
        (fieldBounds.xMax - fieldBounds.xMin) / static_cast<double>(leaders.GetN() + 1);

    for (uint32_t i = 0; i < leaders.GetN(); ++i)
    {
        Ptr<WaypointMobilityModel> waypoint = leaders.Get(i)->GetObject<WaypointMobilityModel>();
        if (!waypoint)
        {
            NS_FATAL_ERROR("WaypointMobilityModel no instalado en lider vertical " << i);
        }

        const double laneX = fieldBounds.xMin + laneSpacing * static_cast<double>(i + 1);
        double t = 0.0;
        double y = yMin;

        waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(laneX, y, 0.0)));
        while (t + legDuration <= simTime + legDuration)
        {
            t += legDuration;
            y = (y == yMin) ? yMax : yMin;
            waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(laneX, y, 0.0)));
        }
    }
}

static void
InstallRandomWaypointLeaders(const NodeContainer& leaders,
                             const Rectangle& fieldBounds,
                             double leaderSpeed,
                             double pauseSeconds)
{
    // Este modo convierte la trayectoria del líder en un proceso aleatorio
    // reproducible dentro del campo. Los followers conservan movilidad relativa
    // respecto de ese líder, así que el cluster sigue existiendo como grupo.
    Ptr<RandomRectanglePositionAllocator> initialPositions =
        CreateUniformPositionAllocator(fieldBounds);
    Ptr<RandomRectanglePositionAllocator> destinationPositions =
        CreateUniformPositionAllocator(fieldBounds);

    MobilityHelper mobility;
    mobility.SetPositionAllocator(initialPositions);
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed",
                              StringValue(MakeConstantRv(std::max(0.5, leaderSpeed))),
                              "Pause",
                              StringValue(MakeConstantRv(std::max(0.0, pauseSeconds))),
                              "PositionAllocator",
                              PointerValue(destinationPositions));
    mobility.Install(leaders);
}

static void
InstallRelativeClusterMobility(const NodeContainer& followers,
                               Ptr<MobilityModel> leaderMobility,
                               const Rectangle& localBounds,
                               double followerSpeedMax)
{
    if (followers.GetN() == 0)
    {
        return;
    }

    // GroupMobilityHelper encapsula la idea de movilidad jerárquica. Según la
    // documentación de ns-3, internamente construye movilidad relativa respecto
    // de un modelo de referencia, lo que aquí representa "movimiento de cluster"
    // y "movimiento de nodo" al mismo tiempo.
    GroupMobilityHelper mobility;
    mobility.SetReferenceMobilityModel(leaderMobility);
    mobility.SetMemberPositionAllocator(CreateUniformPositionAllocator(localBounds));
    mobility.SetMemberMobilityModel("ns3::RandomWalk2dMobilityModel",
                                    "Bounds",
                                    RectangleValue(localBounds),
                                    "Mode",
                                    EnumValue(RandomWalk2dMobilityModel::MODE_TIME),
                                    "Time",
                                    TimeValue(Seconds(2.0)),
                                    "Speed",
                                    StringValue(MakeUniformRv(0.3, std::max(0.6, followerSpeedMax))));
    mobility.Install(followers);
}

static void
InstallSensorMobility(const NodeContainer& sensors, const Rectangle& fieldBounds)
{
    // Los sensores son estáticos por hipótesis experimental; solo su posición
    // inicial es aleatoria uniforme dentro del campo.
    MobilityHelper mobility;
    mobility.SetPositionAllocator(CreateUniformPositionAllocator(fieldBounds));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensors);
}

static void
InstallFixedMobility(const NodeContainer& nodes, const std::vector<Vector>& positions)
{
    // Se usa para nodos cuya posición debe fijarse explícitamente, como la casa.
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& position : positions)
    {
        positionAlloc->Add(position);
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
}

static uint32_t
FindNearestNode(const Vector& sourcePosition, const NodeContainer& candidates)
{
    // Esta utilidad implementa asignación por proximidad geométrica.
    // Se usa tanto para sensor->agrobot como para emparejar agrobots con UGVs.
    uint32_t bestIndex = 0;
    double bestDistance2 = std::numeric_limits<double>::max();

    for (uint32_t i = 0; i < candidates.GetN(); ++i)
    {
        const Vector candidatePosition = candidates.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const double dx = sourcePosition.x - candidatePosition.x;
        const double dy = sourcePosition.y - candidatePosition.y;
        const double distance2 = dx * dx + dy * dy;

        if (distance2 < bestDistance2)
        {
            bestDistance2 = distance2;
            bestIndex = i;
        }
    }

    return bestIndex;
}

static std::vector<uint32_t>
AssignAgrobotLeadersToUgvClusters(const NodeContainer& agrobotLeaders, const NodeContainer& ugvLeaders)
{
    // Esta asignación balancea la jerarquía entre niveles:
    // primero intenta un matching 1-a-1 por proximidad entre líderes agrobot y
    // líderes UGV; si sobra algún agrobot, lo asigna al UGV más cercano.
    const uint32_t invalidAssignment = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> assignment(agrobotLeaders.GetN(), invalidAssignment);
    std::vector<bool> agrobotTaken(agrobotLeaders.GetN(), false);

    for (uint32_t ugvIndex = 0; ugvIndex < ugvLeaders.GetN(); ++ugvIndex)
    {
        const Vector ugvPosition = ugvLeaders.Get(ugvIndex)->GetObject<MobilityModel>()->GetPosition();
        uint32_t bestAgrobotIndex = invalidAssignment;
        double bestDistance2 = std::numeric_limits<double>::max();

        for (uint32_t agrobotIndex = 0; agrobotIndex < agrobotLeaders.GetN(); ++agrobotIndex)
        {
            if (agrobotTaken[agrobotIndex])
            {
                continue;
            }

            const Vector agrobotPosition =
                agrobotLeaders.Get(agrobotIndex)->GetObject<MobilityModel>()->GetPosition();
            const double dx = agrobotPosition.x - ugvPosition.x;
            const double dy = agrobotPosition.y - ugvPosition.y;
            const double distance2 = dx * dx + dy * dy;

            if (distance2 < bestDistance2)
            {
                bestDistance2 = distance2;
                bestAgrobotIndex = agrobotIndex;
            }
        }

        if (bestAgrobotIndex != invalidAssignment)
        {
            assignment[bestAgrobotIndex] = ugvIndex;
            agrobotTaken[bestAgrobotIndex] = true;
        }
    }

    for (uint32_t agrobotIndex = 0; agrobotIndex < agrobotLeaders.GetN(); ++agrobotIndex)
    {
        if (assignment[agrobotIndex] != invalidAssignment)
        {
            continue;
        }

        const Vector agrobotPosition =
            agrobotLeaders.Get(agrobotIndex)->GetObject<MobilityModel>()->GetPosition();
        assignment[agrobotIndex] = FindNearestNode(agrobotPosition, ugvLeaders);
    }

    return assignment;
}

static StageMetrics
CollectStageMetrics(const std::map<FlowId, FlowMonitor::FlowStats>& stats,
                    const Ptr<Ipv4FlowClassifier>& classifier,
                    uint16_t destinationPort,
                    const std::map<uint32_t, uint32_t>& sourceIndexByIp,
                    std::vector<bool>& sourceDelivered)
{
    // FlowMonitor opera a nivel IP/flujo. Esta función filtra por puerto de
    // destino para reconstruir las tres etapas lógicas del experimento.
    StageMetrics metrics;

    for (const auto& [flowId, st] : stats)
    {
        const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(flowId);
        const bool isUdpDataFlow = (tuple.protocol == 17) && (tuple.destinationPort == destinationPort);
        if (!isUdpDataFlow)
        {
            continue;
        }

        metrics.txPackets += st.txPackets;
        metrics.rxPackets += st.rxPackets;
        metrics.totalDelaySeconds += st.delaySum.GetSeconds();
        metrics.rxDelaySamples += st.rxPackets;

        auto it = sourceIndexByIp.find(tuple.sourceAddress.Get());
        if (it != sourceIndexByIp.end() && st.rxPackets > 0)
        {
            sourceDelivered[it->second] = true;
        }
    }

    return metrics;
}

static double
ComputePdr(const StageMetrics& metrics)
{
    // PDR = paquetes recibidos / paquetes transmitidos para una etapa lógica.
    // Lo mantenemos separado para poder reutilizar la misma fórmula en consola
    // y en el CSV final sin duplicar código.
    if (metrics.txPackets == 0)
    {
        return 0.0;
    }

    return 100.0 * static_cast<double>(metrics.rxPackets) / static_cast<double>(metrics.txPackets);
}

static LayerAggregateStats
AggregateLayerStats(const std::vector<Ptr<StoreCarryForwardApp>>& apps)
{
    // Reduce varias apps homogéneas a la vista mínima de capa usada por el
    // análisis: expiración por TTL y presión promedio de buffer.
    LayerAggregateStats stats;
    for (const auto& app : apps)
    {
        stats.nodeCount++;
        stats.ttlDrops += app->GetTtlDrops();
        stats.averageOccupancySum += app->GetAverageOccupancy();
    }
    return stats;
}

static double
ComputeAverageOccupancy(const LayerAggregateStats& stats)
{
    if (stats.nodeCount == 0)
    {
        return 0.0;
    }
    return stats.averageOccupancySum / static_cast<double>(stats.nodeCount);
}

static bool
FileHasContent(const std::string& fileName)
{
    // Sirve para no reescribir encabezados CSV cuando una campaña de barrido
    // acumula varias corridas en el mismo archivo.
    std::ifstream input(fileName);
    return input.good() && input.peek() != std::ifstream::traits_type::eof();
}

static void
EnsureCsvHeader(const std::string& fileName, const std::string& headerLine)
{
    // Solo crea el encabezado cuando el archivo no existe o está vacío.
    // Esto preserva resultados anteriores y facilita barridos reproducibles.
    if (FileHasContent(fileName))
    {
        return;
    }

    std::ofstream output(fileName, std::ios::out);
    output << headerLine << "\n";
}

static void
AppendCsvLine(const std::string& fileName, const std::string& line)
{
    // Todas las exportaciones se hacen por append para no sobrescribir artefactos
    // previos sin instrucción explícita del usuario.
    std::ofstream output(fileName, std::ios::app);
    output << line << "\n";
}

static std::string
FormatDouble(double value, uint32_t precision = 6)
{
    // Formato estable para CSV: evita notación científica y mantiene columnas
    // consistentes al postprocesar en Python, R o Excel.
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

static std::string
JoinCsvFields(const std::vector<std::string>& fields)
{
    // Ensambla una fila CSV simple. No escapamos comas porque los campos que
    // exporta este experimento son numéricos o etiquetas sin separadores.
    std::ostringstream oss;
    for (size_t i = 0; i < fields.size(); ++i)
    {
        if (i > 0)
        {
            oss << ",";
        }
        oss << fields[i];
    }
    return oss.str();
}

static std::string
BuildDefaultRunLabel(const std::string& labelPrefix, uint32_t rngRun)
{
    // Etiqueta por defecto para distinguir corridas sin exigir un nombre manual.
    // Se usa tanto en CSV resumen como en serie temporal.
    std::ostringstream oss;
    oss << labelPrefix << "-run" << rngRun;
    return oss.str();
}

static uint32_t
CountCoveredSensorsAtAgrobotLevel(const std::vector<Ptr<StoreCarryForwardApp>>& agrobotRelayApps)
{
    // Agrega la cobertura del primer nivel sumando las fuentes únicas vistas por
    // cada relay agrobot. Es una cobertura "hasta nivel 1", distinta de la casa.
    uint32_t coveredSensors = 0;
    for (const auto& app : agrobotRelayApps)
    {
        coveredSensors += app->GetUniqueSourceCount();
    }
    return coveredSensors;
}

} // namespace

int
main(int argc, char* argv[])
{
    // Reglas mínimas de validez que protegen a la simulación de configuraciones
    // imposibles o inconsistentes con la API de ns-3.
    const uint32_t minClusterNodes = 2;
    const uint32_t nLevel1Clusters = 3;
    const uint32_t nLevel2Clusters = 2;

    // -------------------------------------------------------------------------
    // Parametros de experimento
    // -------------------------------------------------------------------------
    // Agrobots: 3 clusters que recolectan datos de sensores.
    // UGVs: 2 clusters terrestres que reciben desde los lideres agrobot.
    // Casa: sumidero fijo al centro de la finca.
    // Toda la conectividad usa una sola MANET Wi-Fi ad hoc con AODV.
    // El experimento usa exclusivamente store-carry-forward.
    // -------------------------------------------------------------------------
    double simTime = 1001.0;
    uint32_t agrobotsPerCluster = 5;
    uint32_t level2NodesPerCluster = 3;
    uint32_t sensorsPerField = 24;
    double fieldWidth = 240.0;
    double fieldHeight = 180.0;
    double leaderSpeed = 2.0;
    double agrobotLeaderPause = 0.0;
    double followerSpeedMax = 1.2;
    double level2LeaderSpeed = 1.4;
    double level2FollowerSpeedMax = 0.8;
    double clusterBoxWidth = 30.0;
    double clusterBoxHeight = 20.0;
    double level2BoxWidth = 12.0;
    double level2BoxHeight = 8.0;
    double sensorInterval = 3.0;
    uint32_t packetSize = 128;
    uint32_t rngRun = 1;
    std::string flowmonFile = "flowmon-adhoc.xml";
    double sensorContactRange = 35.0;
    double level1ToLevel2ContactRange = 30.0;
    double level2ToHouseContactRange = 50.0;
    double contactCheckInterval = 1.0;
    uint32_t forwardBurstPackets = 6;
    uint32_t bufferCapacityPackets = 128;
    double sampleTtl = 120.0;
    double aoiSamplePeriod = 1.0;
    double metricsSampleInterval = 1.0;
    std::string summaryCsvFile = "summary-adhoc.csv";
    std::string timeseriesCsvFile = "timeseries-adhoc.csv";
    std::string metricsRunLabel;

    // CommandLine expone todos los parámetros experimentales como CLI.
    // Esto sigue la práctica habitual en ejemplos de ns-3: una sola simulación
    // reutilizable con distintos valores sin recompilar.
    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Tiempo total de simulacion en segundos", simTime);
    cmd.AddValue("agrobotsPerCluster",
                 "Cantidad de nodos por cluster agrobot (incluye al lider)",
                 agrobotsPerCluster);
    cmd.AddValue("level2NodesPerCluster",
                 "Cantidad de nodos por cluster UGV (incluye al lider)",
                 level2NodesPerCluster);
    cmd.AddValue("sensorsPerField", "Cantidad de sensores estaticos en el campo", sensorsPerField);
    cmd.AddValue("fieldWidth", "Ancho del campo (m)", fieldWidth);
    cmd.AddValue("fieldHeight", "Alto del campo (m)", fieldHeight);
    cmd.AddValue("leaderSpeed",
                 "Velocidad del lider agrobot (m/s); en random-waypoint se usa como velocidad constante",
                 leaderSpeed);
    cmd.AddValue("agrobotLeaderPause",
                 "Pausa del lider agrobot en random-waypoint (s)",
                 agrobotLeaderPause);
    cmd.AddValue("followerSpeedMax",
                 "Velocidad maxima de seguidores agrobot (m/s)",
                 followerSpeedMax);
    cmd.AddValue("level2LeaderSpeed", "Velocidad del lider UGV (m/s)", level2LeaderSpeed);
    cmd.AddValue("level2FollowerSpeedMax",
                 "Velocidad maxima de seguidores UGV (m/s)",
                 level2FollowerSpeedMax);
    cmd.AddValue("clusterBoxWidth", "Ancho de la caja local del cluster agrobot (m)", clusterBoxWidth);
    cmd.AddValue("clusterBoxHeight", "Alto de la caja local del cluster agrobot (m)", clusterBoxHeight);
    cmd.AddValue("level2BoxWidth", "Ancho de la caja local del cluster UGV (m)", level2BoxWidth);
    cmd.AddValue("level2BoxHeight", "Alto de la caja local del cluster UGV (m)", level2BoxHeight);
    cmd.AddValue("sensorInterval", "Periodo de muestreo/generacion por sensor (s)", sensorInterval);
    cmd.AddValue("packetSize", "Tamano de paquete de cada muestra SCF (bytes)", packetSize);
    cmd.AddValue("rngRun", "Indice de corrida RNG para reproducibilidad", rngRun);
    cmd.AddValue("flowmonFile", "Archivo XML de salida para FlowMonitor", flowmonFile);
    cmd.AddValue("sensorContactRange",
                 "Umbral de contacto sensor->lider agrobot para store-carry-forward (m)",
                 sensorContactRange);
    cmd.AddValue("level1ToLevel2ContactRange",
                 "Umbral de contacto lider agrobot -> lider UGV para store-carry-forward (m)",
                 level1ToLevel2ContactRange);
    cmd.AddValue("level2ToHouseContactRange",
                 "Umbral de contacto lider UGV -> casa para store-carry-forward (m)",
                 level2ToHouseContactRange);
    cmd.AddValue("contactCheckInterval",
                 "Periodo de chequeo de oportunidad de contacto en store-carry-forward (s)",
                 contactCheckInterval);
    cmd.AddValue("forwardBurstPackets",
                 "Maximo de muestras reenviadas por cada oportunidad de contacto",
                 forwardBurstPackets);
    cmd.AddValue("bufferCapacityPackets", "Capacidad de buffer por nodo store-carry-forward", bufferCapacityPackets);
    cmd.AddValue("sampleTtl", "TTL de cada muestra en store-carry-forward (s)", sampleTtl);
    cmd.AddValue("aoiSamplePeriod", "Periodo de muestreo de Age of Information en la casa (s)", aoiSamplePeriod);
    cmd.AddValue("metricsSampleInterval",
                 "Periodo de muestreo para exportar la serie temporal de metricas (s)",
                 metricsSampleInterval);
    cmd.AddValue("summaryCsvFile", "Archivo CSV de resumen final de la corrida", summaryCsvFile);
    cmd.AddValue("timeseriesCsvFile", "Archivo CSV de serie temporal de metricas", timeseriesCsvFile);
    cmd.AddValue("metricsRunLabel", "Etiqueta de corrida para exportacion CSV", metricsRunLabel);
    cmd.Parse(argc, argv);

    const uint32_t minScfPacketSize = SampleHeader::SERIALIZED_SIZE;
    if (metricsRunLabel.empty())
    {
        metricsRunLabel = BuildDefaultRunLabel("scf", rngRun);
    }

    // Validación temprana para fallar con un mensaje claro antes de crear nodos,
    // sockets o eventos. Esto ayuda mucho al depurar campañas de barrido.
    if (agrobotsPerCluster < minClusterNodes || level2NodesPerCluster < minClusterNodes ||
        sensorsPerField < 1 || simTime <= 5.0 || fieldWidth <= 20.0 || fieldHeight <= 20.0 ||
        clusterBoxWidth <= 1.0 || clusterBoxHeight <= 1.0 || level2BoxWidth <= 1.0 ||
        level2BoxHeight <= 1.0 || sensorInterval <= 0.0 || contactCheckInterval <= 0.0 ||
        forwardBurstPackets < 1 || bufferCapacityPackets < 1 || sampleTtl <= 0.0 ||
        aoiSamplePeriod <= 0.0 || metricsSampleInterval <= 0.0)
    {
        std::cerr << "Parametros invalidos para el experimento." << std::endl;
        return 1;
    }

    // Nuestro SampleHeader debe caber completo dentro del paquete de cada muestra.
    if (packetSize < minScfPacketSize)
    {
        std::cerr << "packetSize debe ser >= " << minScfPacketSize
                  << " bytes para alojar SampleHeader." << std::endl;
        return 1;
    }

    if (agrobotLeaderPause < 0.0)
    {
        std::cerr << "agrobotLeaderPause no puede ser negativo." << std::endl;
        return 1;
    }

    // RngSeedManager::SetRun cambia la subsecuencia aleatoria manteniendo la semilla
    // global. Es la forma estándar de ns-3 para generar réplicas reproducibles.
    RngSeedManager::SetRun(rngRun);

    // Derivamos tamaños totales y geometría del escenario a partir de la CLI.
    const uint32_t agrobotFollowersPerCluster = agrobotsPerCluster - 1;
    const uint32_t ugvFollowersPerCluster = level2NodesPerCluster - 1;
    const uint32_t totalAgrobotNodes = nLevel1Clusters * agrobotsPerCluster;
    const uint32_t totalUgvNodes = nLevel2Clusters * level2NodesPerCluster;
    const Rectangle fieldBounds(0.0, fieldWidth, 0.0, fieldHeight);
    const Rectangle agrobotLocalBounds(-clusterBoxWidth * 0.5,
                                       clusterBoxWidth * 0.5,
                                       -clusterBoxHeight * 0.5,
                                       clusterBoxHeight * 0.5);
    const Rectangle ugvLocalBounds(-level2BoxWidth * 0.5,
                                   level2BoxWidth * 0.5,
                                   -level2BoxHeight * 0.5,
                                   level2BoxHeight * 0.5);
    const Vector housePosition(fieldWidth * 0.5, fieldHeight * 0.5, 0.0);

    // Cada conjunto se crea por separado para poder asignarle movilidad, potencia
    // y aplicaciones distintas según su rol en la jerarquía.
    NodeContainer agrobotLeaders;
    agrobotLeaders.Create(nLevel1Clusters);

    NodeContainer agrobotFollowers;
    agrobotFollowers.Create(nLevel1Clusters * agrobotFollowersPerCluster);

    NodeContainer ugvLeaders;
    ugvLeaders.Create(nLevel2Clusters);

    NodeContainer ugvFollowers;
    ugvFollowers.Create(nLevel2Clusters * ugvFollowersPerCluster);

    NodeContainer sensors;
    sensors.Create(sensorsPerField);

    NodeContainer house;
    house.Create(1);

    // NodeContainer permite agrupar nodos sin copiar objetos; es la abstracción
    // base de ns-3 para instalar helpers sobre subconjuntos del escenario.
    NodeContainer agrobotNodes;
    agrobotNodes.Add(agrobotLeaders);
    agrobotNodes.Add(agrobotFollowers);

    NodeContainer ugvNodes;
    ugvNodes.Add(ugvLeaders);
    ugvNodes.Add(ugvFollowers);

    NodeContainer allNodes;
    allNodes.Add(agrobotNodes);
    allNodes.Add(ugvNodes);
    allNodes.Add(sensors);
    allNodes.Add(house);

    // Reconstruimos explícitamente la partición por cluster para instalar
    // movilidad relativa y reportar métricas por grupo.
    std::vector<NodeContainer> agrobotClusterFollowers(nLevel1Clusters);
    std::vector<NodeContainer> ugvClusterFollowers(nLevel2Clusters);
    for (uint32_t c = 0; c < nLevel1Clusters; ++c)
    {
        for (uint32_t j = 0; j < agrobotFollowersPerCluster; ++j)
        {
            agrobotClusterFollowers[c].Add(
                agrobotFollowers.Get(c * agrobotFollowersPerCluster + j));
        }
    }

    for (uint32_t c = 0; c < nLevel2Clusters; ++c)
    {
        for (uint32_t j = 0; j < ugvFollowersPerCluster; ++j)
        {
            ugvClusterFollowers[c].Add(ugvFollowers.Get(c * ugvFollowersPerCluster + j));
        }
    }

    // -------------------------------------------------------------------------
    // MOVILIDAD
    // -------------------------------------------------------------------------
    // Ambos niveles preservan movilidad de cluster y de nodo.
    // Agrobots: lideres con movilidad aleatoria; seguidores con movilidad
    // relativa.
    // UGVs: lideres con barrido vertical; seguidores con movilidad relativa.
    // -------------------------------------------------------------------------
    // MobilityHelper simplifica instalación masiva de modelos de movilidad.
    // Aquí lo usamos solo para los líderes; luego cada follower se ancla al líder.
    InstallRandomWaypointLeaders(
        agrobotLeaders, fieldBounds, leaderSpeed, agrobotLeaderPause);

    for (uint32_t c = 0; c < nLevel1Clusters; ++c)
    {
        InstallRelativeClusterMobility(agrobotClusterFollowers[c],
                                       agrobotLeaders.Get(c)->GetObject<MobilityModel>(),
                                       agrobotLocalBounds,
                                       followerSpeedMax);
    }

    MobilityHelper ugvLeaderMobility;
    ugvLeaderMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    ugvLeaderMobility.Install(ugvLeaders);
    ConfigureSweepWaypoints(ugvLeaders, simTime, fieldBounds, level2LeaderSpeed, false);

    for (uint32_t c = 0; c < nLevel2Clusters; ++c)
    {
        InstallRelativeClusterMobility(ugvClusterFollowers[c],
                                       ugvLeaders.Get(c)->GetObject<MobilityModel>(),
                                       ugvLocalBounds,
                                       level2FollowerSpeedMax);
    }

    InstallSensorMobility(sensors, fieldBounds);
    InstallFixedMobility(house, {housePosition});

    // -------------------------------------------------------------------------
    // WiFi ad-hoc + AODV
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // CAPA FÍSICA Y MAC
    // -------------------------------------------------------------------------
    // WifiHelper::SetStandard fija el estándar 802.11g, que en ns-3 configura
    // banda de 2.4 GHz y ancho de canal de 20 MHz, suficiente para una MANET
    // simple sin la complejidad extra de 802.11n/ac.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    // ConstantRateWifiManager fuerza tasas fijas de datos/control.
    // Se eligió para que los cambios observados provengan más de movilidad y SCF
    // que de adaptación automática de tasa.
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate12Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps"));

    WifiMacHelper mac;
    // AdhocWifiMac construye una red sin AP ni infraestructura, coherente con la
    // hipótesis MANET del taller.
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiChannelHelper channelHelper;
    // El retardo de propagación a velocidad constante es la opción estándar del
    // helper Yans para canal inalámbrico simple.
    channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // LogDistancePropagationLossModel con ReferenceLoss=40.046 calibra la pérdida
    // de referencia a 2.4 GHz, alineada con 802.11g según ejemplos de ns-3.
    channelHelper.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                     "ReferenceLoss",
                                     DoubleValue(40.046));
    Ptr<YansWifiChannel> wifiChannel = channelHelper.Create();

    // Usamos PHY distintos por rol solo para fijar potencias TX diferentes sin
    // crear canales separados: todos comparten el mismo medio inalámbrico.
    YansWifiPhyHelper agrobotPhy;
    agrobotPhy.SetChannel(wifiChannel);
    agrobotPhy.Set("TxPowerStart", DoubleValue(16.0));
    agrobotPhy.Set("TxPowerEnd", DoubleValue(16.0));

    YansWifiPhyHelper ugvPhy;
    ugvPhy.SetChannel(wifiChannel);
    ugvPhy.Set("TxPowerStart", DoubleValue(18.0));
    ugvPhy.Set("TxPowerEnd", DoubleValue(18.0));

    YansWifiPhyHelper sensorPhy;
    sensorPhy.SetChannel(wifiChannel);
    sensorPhy.Set("TxPowerStart", DoubleValue(12.0));
    sensorPhy.Set("TxPowerEnd", DoubleValue(12.0));

    YansWifiPhyHelper housePhy;
    housePhy.SetChannel(wifiChannel);
    housePhy.Set("TxPowerStart", DoubleValue(20.0));
    housePhy.Set("TxPowerEnd", DoubleValue(20.0));

    // wifi.Install materializa los WifiNetDevice en cada grupo de nodos.
    NetDeviceContainer agrobotDevices = wifi.Install(agrobotPhy, mac, agrobotNodes);
    NetDeviceContainer ugvDevices = wifi.Install(ugvPhy, mac, ugvNodes);
    NetDeviceContainer sensorDevices = wifi.Install(sensorPhy, mac, sensors);
    NetDeviceContainer houseDevices = wifi.Install(housePhy, mac, house);

    // -------------------------------------------------------------------------
    // PILA IP Y ROUTING
    // -------------------------------------------------------------------------
    // AodvHelper se eligió porque la documentación de ns-3 lo posiciona como un
    // protocolo MANET reactivo apropiado cuando la conectividad cambia por movilidad.
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    // Ipv4AddressHelper asigna direcciones secuenciales sobre una sola subred,
    // suficiente porque el experimento comparte una única MANET ad hoc.
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");

    NetDeviceContainer allDevices;
    allDevices.Add(agrobotDevices);
    allDevices.Add(ugvDevices);
    allDevices.Add(sensorDevices);
    allDevices.Add(houseDevices);
    Ipv4InterfaceContainer ifaces = ipv4.Assign(allDevices);

    // Guardamos offsets dentro de Ipv4InterfaceContainer para recuperar direcciones
    // por rol sin depender de búsquedas posteriores.
    const uint32_t ugvLeaderStartIndex = totalAgrobotNodes;
    const uint32_t sensorStartIndex = totalAgrobotNodes + totalUgvNodes;
    const uint32_t houseIndex = sensorStartIndex + sensorsPerField;

    // Separamos etapas lógicas por puerto UDP para que FlowMonitor pueda medirlas
    // por separado a pesar de compartir la misma red IP.
    const uint16_t sensorToL1Port = 9000;
    const uint16_t level1ToLevel2Port = 9001;
    const uint16_t level2ToHousePort = 9002;

    // Estas tablas enlazan roles experimentales con direcciones IP concretas.
    std::vector<Ipv4Address> agrobotLeaderIps;
    std::vector<Ipv4Address> ugvLeaderIps;
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        agrobotLeaderIps.push_back(ifaces.GetAddress(i));
    }
    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        ugvLeaderIps.push_back(ifaces.GetAddress(ugvLeaderStartIndex + i));
    }
    const Ipv4Address houseIp = ifaces.GetAddress(houseIndex);

    std::vector<uint32_t> sensorAssignedAgrobotCluster(sensorsPerField);
    std::vector<uint32_t> sensorsPerAgrobotCluster(nLevel1Clusters, 0);
    std::vector<uint32_t> agrobotAssignedUgvCluster(nLevel1Clusters);
    std::vector<uint32_t> agrobotLeadersPerUgvCluster(nLevel2Clusters, 0);

    std::map<uint32_t, uint32_t> sensorIndexByIp;
    std::map<uint32_t, uint32_t> agrobotLeaderIndexByIp;
    std::map<uint32_t, uint32_t> ugvLeaderIndexByIp;

    // Emparejamiento entre niveles: cada líder agrobot tiene un cluster UGV
    // destino para el segundo relevo.
    const std::vector<uint32_t> balancedAssignments =
        AssignAgrobotLeadersToUgvClusters(agrobotLeaders, ugvLeaders);
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        agrobotLeaderIndexByIp.emplace(agrobotLeaderIps[i].Get(), i);
        agrobotAssignedUgvCluster[i] = balancedAssignments[i];
        agrobotLeadersPerUgvCluster[agrobotAssignedUgvCluster[i]]++;
    }

    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        ugvLeaderIndexByIp.emplace(ugvLeaderIps[i].Get(), i);
    }

    // Cada sensor se asigna al líder agrobot más cercano al inicio de la corrida.
    // Esa decisión fija el camino lógico sensor -> nivel 1 para toda la réplica.
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        const Vector sensorPosition = sensors.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const uint32_t agrobotClusterIndex = FindNearestNode(sensorPosition, agrobotLeaders);
        sensorAssignedAgrobotCluster[i] = agrobotClusterIndex;
        sensorsPerAgrobotCluster[agrobotClusterIndex]++;
        sensorIndexByIp.emplace(ifaces.GetAddress(sensorStartIndex + i).Get(), i);
    }

    // FlowMonitor instrumenta todos los nodos a nivel IP/flujo.
    // Se usa para medir las tres etapas lógicas; las métricas SCF finas siguen
    // calculándose manualmente a nivel de aplicación.
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    std::vector<Ptr<StoreCarryForwardApp>> sensorStoreApps;
    std::vector<Ptr<StoreCarryForwardApp>> agrobotRelayApps;
    std::vector<Ptr<StoreCarryForwardApp>> ugvRelayApps;
    Ptr<HouseCollectorApp> houseCollectorApp;
    // -------------------------------------------------------------------------
    // STORE-CARRY-FORWARD
    // -------------------------------------------------------------------------
    // Sensores, líderes agrobot y líderes UGV reciben apps distintas pero la
    // misma clase base StoreCarryForwardApp para mantener una semántica única:
    // recibir, almacenar, transportar y reenviar por contactos.
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("sensor");
        app->SetForwardHop(0);
        app->SetNextHop(agrobotLeaders.Get(sensorAssignedAgrobotCluster[i]),
                        agrobotLeaderIps[sensorAssignedAgrobotCluster[i]],
                        sensorToL1Port);
        app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                            Seconds(sampleTtl),
                                            Seconds(contactCheckInterval),
                                            forwardBurstPackets,
                                            sensorContactRange);
        app->SetSourceParameters(i,
                                 Seconds(sensorInterval),
                                 packetSize,
                                 Seconds(1.5 + 0.05 * static_cast<double>(i % 20)));
        sensors.Get(i)->AddApplication(app);
        // Start/StopTime siguen la convención de ns-3 para que la aplicación
        // entre al ciclo de vida del simulador en lugar de arrancar "a mano".
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(simTime - 0.5));
        sensorStoreApps.push_back(app);
    }

    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("agrobot-relay");
        app->SetListenPort(sensorToL1Port);
        app->SetForwardHop(1);
        app->SetNextHop(ugvLeaders.Get(agrobotAssignedUgvCluster[i]),
                        ugvLeaderIps[agrobotAssignedUgvCluster[i]],
                        level1ToLevel2Port);
        app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                            Seconds(sampleTtl),
                                            Seconds(contactCheckInterval),
                                            forwardBurstPackets,
                                            level1ToLevel2ContactRange);
        agrobotLeaders.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(simTime - 0.25));
        agrobotRelayApps.push_back(app);
    }

    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("ugv-relay");
        app->SetListenPort(level1ToLevel2Port);
        app->SetForwardHop(2);
        app->SetNextHop(house.Get(0), houseIp, level2ToHousePort);
        app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                            Seconds(sampleTtl),
                                            Seconds(contactCheckInterval),
                                            forwardBurstPackets,
                                            level2ToHouseContactRange);
        ugvLeaders.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(simTime - 0.15));
        ugvRelayApps.push_back(app);
    }

    houseCollectorApp = CreateObject<HouseCollectorApp>();
    // La casa solo escucha en el último puerto y calcula métricas extremo a extremo.
    houseCollectorApp->Configure(level2ToHousePort, sensorsPerField, Seconds(aoiSamplePeriod));
    house.Get(0)->AddApplication(houseCollectorApp);
    houseCollectorApp->SetStartTime(Seconds(0.5));
    houseCollectorApp->SetStopTime(Seconds(simTime));

    // -------------------------------------------------------------------------
    // EXPORTACION TEMPORAL
    // -------------------------------------------------------------------------
    // La serie temporal queda acotada a las métricas que alimentan los
    // dashboards: cobertura intermedia/final y frescura observada en la casa.
    std::vector<std::string> timeseriesHeaderFields = {"run_label",
                                                       "time_s",
                                                       "agrobot_coverage_pct",
                                                       "house_coverage_pct",
                                                       "house_aoi_avg_s"};

    EnsureCsvHeader(timeseriesCsvFile, JoinCsvFields(timeseriesHeaderFields));

    // snapshotMetrics captura el estado observado en un instante fijo.
    // Se programa periódicamente y escribe una fila por muestra temporal.
    std::function<void()> snapshotMetrics;
    snapshotMetrics = [&]() {
        const double nowSeconds = Simulator::Now().GetSeconds();

        std::vector<std::string> row = {metricsRunLabel, FormatDouble(nowSeconds, 3)};

        const uint32_t coveredSensorsAtAgrobotLevel =
            CountCoveredSensorsAtAgrobotLevel(agrobotRelayApps);
        const uint32_t deliveredSensorsAtHouse = houseCollectorApp->GetSensorsDeliveredToHouse();

        row.push_back(FormatDouble(100.0 * static_cast<double>(coveredSensorsAtAgrobotLevel) /
                                       static_cast<double>(sensorsPerField),
                                   4));
        row.push_back(FormatDouble(100.0 * static_cast<double>(deliveredSensorsAtHouse) /
                                       static_cast<double>(sensorsPerField),
                                   4));
        row.push_back(FormatDouble(houseCollectorApp->GetCurrentAverageAoi(), 6));

        AppendCsvLine(timeseriesCsvFile, JoinCsvFields(row));

        if (nowSeconds + metricsSampleInterval <= simTime)
        {
            Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);
        }
    };

    Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);

    // La corrida termina exactamente en simTime; las apps ya tienen StopTime algo
    // anterior para dejar un margen de drenaje antes del cierre global.
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // CheckForLostPackets fuerza a FlowMonitor a contabilizar pérdidas pendientes
    // antes de que leamos las estadísticas finales.
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    const std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::vector<bool> agrobotLeadersDelivered(nLevel1Clusters, false);
    std::vector<bool> ugvLeadersDelivered(nLevel2Clusters, false);

    // Reconstruimos las etapas lógicas que luego se exponen en los dashboards
    // finales. La etapa 1 sigue disponible vía cobertura temporal agrobot.
    const StageMetrics stage2Metrics =
        CollectStageMetrics(stats,
                            classifier,
                            level1ToLevel2Port,
                            agrobotLeaderIndexByIp,
                            agrobotLeadersDelivered);
    const StageMetrics stage3Metrics =
        CollectStageMetrics(
            stats, classifier, level2ToHousePort, ugvLeaderIndexByIp, ugvLeadersDelivered);

    // -------------------------------------------------------------------------
    // REPORTE EN CONSOLA
    // -------------------------------------------------------------------------
    // El log queda alineado con los dashboards para no exportar telemetría que
    // luego no se usa en el análisis del taller.
    std::cout << "\n===== RESULTADOS MANET JERARQUICA AGROBOTS Y UGV =====\n";
    std::cout << "Run label: " << metricsRunLabel << ", rngRun=" << rngRun << "\n";
    std::cout << "simTime=" << simTime << " s, metricsSampleInterval=" << metricsSampleInterval
              << " s\n";
    std::cout << "TTL=" << sampleTtl << " s, buffer=" << bufferCapacityPackets
              << " muestras por nodo\n";
    std::cout << "Rangos de contacto (m): sensor->agrobot=" << sensorContactRange
              << ", agrobot->UGV=" << level1ToLevel2ContactRange
              << ", UGV->casa=" << level2ToHouseContactRange << "\n";

    const LayerAggregateStats sensorLayerStats = AggregateLayerStats(sensorStoreApps);
    const LayerAggregateStats agrobotLayerStats = AggregateLayerStats(agrobotRelayApps);
    const LayerAggregateStats ugvLayerStats = AggregateLayerStats(ugvRelayApps);

    std::cout << "\n[Metricas finales]\n";
    std::cout << "Cobertura final en casa (%): "
              << (100.0 * static_cast<double>(houseCollectorApp->GetSensorsDeliveredToHouse()) /
                  static_cast<double>(sensorsPerField))
              << "\n";
    std::cout << "Delay extremo a extremo promedio (s): "
              << houseCollectorApp->GetAverageEndToEndDelay() << "\n";
    std::cout << "Age of Information promedio (s): " << houseCollectorApp->GetAverageAoi()
              << "\n";
    std::cout << "PDR Stage 2 (%): " << ComputePdr(stage2Metrics) << "\n";
    std::cout << "PDR Stage 3 (%): " << ComputePdr(stage3Metrics) << "\n";

    std::cout << "\n[SCF buffers y expiracion]\n";
    std::cout << "Sensores -> buffer medio: " << ComputeAverageOccupancy(sensorLayerStats)
              << ", TTL drops: " << sensorLayerStats.ttlDrops << "\n";
    std::cout << "Agrobots -> buffer medio: " << ComputeAverageOccupancy(agrobotLayerStats)
              << ", TTL drops: " << agrobotLayerStats.ttlDrops << "\n";
    std::cout << "UGVs -> buffer medio: " << ComputeAverageOccupancy(ugvLayerStats)
              << ", TTL drops: " << ugvLayerStats.ttlDrops << "\n";

    std::cout << "FlowMonitor XML: " << flowmonFile << "\n";

    // El XML conserva trazabilidad detallada por flujo para análisis posterior.
    monitor->SerializeToXmlFile(flowmonFile, true, true);

    // -------------------------------------------------------------------------
    // CSV RESUMEN
    // -------------------------------------------------------------------------
    // Una fila por corrida. Se exporta solo el subconjunto que consumen los
    // dashboards comparativos de rango, velocidad y TTL+buffer.
    std::vector<std::string> summaryHeaderFields = {"run_label",
                                                    "rng_run",
                                                    "sim_time_s",
                                                    "metrics_sample_interval_s",
                                                    "stage2_pdr_pct",
                                                    "stage3_pdr_pct",
                                                    "house_coverage_pct",
                                                    "house_e2e_delay_avg_s",
                                                    "house_aoi_avg_s",
                                                    "sensor_ttl_drops",
                                                    "sensor_avg_buffer_occ",
                                                    "agrobot_ttl_drops",
                                                    "agrobot_avg_buffer_occ",
                                                    "ugv_ttl_drops",
                                                    "ugv_avg_buffer_occ"};

    EnsureCsvHeader(summaryCsvFile, JoinCsvFields(summaryHeaderFields));

    // La fila se arma en el mismo orden que el header para que el archivo pueda
    // consumirse sin inferencias externas.
    std::vector<std::string> summaryRow = {metricsRunLabel,
                                           std::to_string(rngRun),
                                           FormatDouble(simTime, 3),
                                           FormatDouble(metricsSampleInterval, 3),
                                           FormatDouble(ComputePdr(stage2Metrics), 6),
                                           FormatDouble(ComputePdr(stage3Metrics), 6),
                                           FormatDouble(100.0 * static_cast<double>(houseCollectorApp->GetSensorsDeliveredToHouse()) /
                                                            static_cast<double>(sensorsPerField),
                                                        6),
                                           FormatDouble(houseCollectorApp->GetAverageEndToEndDelay(), 6),
                                           FormatDouble(houseCollectorApp->GetAverageAoi(), 6),
                                           std::to_string(sensorLayerStats.ttlDrops),
                                           FormatDouble(ComputeAverageOccupancy(sensorLayerStats), 6),
                                           std::to_string(agrobotLayerStats.ttlDrops),
                                           FormatDouble(ComputeAverageOccupancy(agrobotLayerStats), 6),
                                           std::to_string(ugvLayerStats.ttlDrops),
                                           FormatDouble(ComputeAverageOccupancy(ugvLayerStats), 6)};

    AppendCsvLine(summaryCsvFile, JoinCsvFields(summaryRow));

    // Destroy libera objetos ns-3 y deja la corrida lista para el siguiente run.
    Simulator::Destroy();
    return 0;
}
