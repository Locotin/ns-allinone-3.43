// Importación de módulos principales del simulador ns-3 requeridos para este experimento.
#include "ns3/aodv-module.h"          // Provee el protocolo de enrutamiento reactivo AODV para redes Ad-Hoc.
#include "ns3/applications-module.h"  // Permite heredar y crear aplicaciones de red personalizadas (clase Application).
#include "ns3/core-module.h"          // Núcleo de ns-3: manejo del tiempo (Simulator), eventos, callbacks, punteros inteligentes (Ptr) y logs.
#include "ns3/flow-monitor-module.h"  // Herramienta analítica para medir rendimiento a nivel IP (paquetes, delays, pérdidas).
#include "ns3/internet-module.h"      // Implementación de la pila TCP/IP (IPv4, enrutamiento, sockets UDP).
#include "ns3/mobility-module.h"      // Modelos de movimiento y posicionamiento geométrico para los nodos.
#include "ns3/network-module.h"       // Estructuras fundamentales de red: Nodos, NetDevices, Paquetes, Headers y Sockets.
#include "ns3/wifi-module.h"          // Implementación de la capa Física (PHY) y Enlace (MAC) del estándar IEEE 802.11 (Wi-Fi).

// Librerías estándar de C++ utilizadas para estructuras de datos y utilidades.
#include <algorithm>      // Para algoritmos estándar como std::max, std::count.
#include <cstdint>        // Para tipos de enteros de tamaño fijo (uint32_t, uint64_t, uint8_t).
#include <deque>          // Cola doblemente terminada, ideal para el buffer Store-Carry-Forward (FIFO).
#include <fstream>        // Operaciones de lectura y escritura de archivos (para los CSV).
#include <functional>     // Para usar std::function (funciones lambda, callbacks).
#include <iomanip>        // Para manipulación de formato de salida (std::setprecision).
#include <iostream>       // Para impresión en consola (std::cout, std::cerr).
#include <limits>         // Para obtener valores máximos de los tipos de datos (std::numeric_limits).
#include <map>            // Estructura de diccionario ordenado (clave-valor).
#include <sstream>        // Para construir cadenas de texto dinámicamente con variables.
#include <string>         // Manejo de cadenas de texto.
#include <unordered_set>  // Conjuntos sin ordenar (hash tables) para búsquedas ultrarrápidas de duplicados.
#include <vector>         // Arreglos dinámicos para almacenar listas de elementos.

using namespace ns3; // Evita tener que escribir "ns3::" antes de cada clase del simulador.

// Define un componente de log para ns-3. Permite activar mensajes de depuración específicos
// desde la consola de comandos usando la variable de entorno NS_LOG.
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
//
// Contrato experimental resumido:
// - Sensores estáticos generan muestras periódicas y nunca entregan directo a casa.
// - Agrobots realizan el primer relevo y UGVs el segundo relevo hacia la casa.
// - Los seguidores no se mueven libremente por todo el campo: su movilidad se
//   define relativa al líder del cluster.
// - El forwarding store-carry-forward se habilita por contacto geométrico entre
//   nodos, mientras que la entrega real viaja sobre la pila IP/UDP estándar de ns-3.
//
// Supuestos y limitaciones metodológicas:
// - El "contacto" se aproxima con distancia euclideana y umbrales fijos; no se
//   modelan RSSI, SNR adaptativa, fading ni shadowing del cultivo.
// - FlowMonitor mide flujos IP/UDP por etapa, pero no observa el estado interno
//   de los buffers SCF; esas métricas salen de cálculo manual en las apps.
// - El AoI se calcula en la casa usando la actualización más fresca observada por
//   sensor; paquetes en tránsito o aún retenidos no cuentan como información disponible.
// -----------------------------------------------------------------------------

// StageMetrics resume lo que FlowMonitor reporta por cada salto lógico.
// No modela SCF por sí mismo; solo sintetiza TX/RX y delay IP/UDP por etapa.
// Estructura de datos para agrupar métricas de red a nivel de enlace/flujo.
struct StageMetrics
{
    uint64_t txPackets{0};         // Cantidad de paquetes transmitidos en la etapa.
    uint64_t rxPackets{0};         // Cantidad de paquetes exitosamente recibidos.
    uint64_t rxDelaySamples{0};    // Número de muestras usadas para calcular el delay promedio.
    double totalDelaySeconds{0.0}; // Sumatoria del delay de todos los paquetes recibidos (para promediar después).
};

// BufferedSample representa una muestra almacenada localmente en un nodo SCF.
// Guardamos el paquete y metadatos suficientes para medir TTL, carry time
// y deduplicación por sensor/sampleId.
struct BufferedSample
{
    Ptr<Packet> packet;            // Puntero inteligente al paquete de ns-3 almacenado en el buffer.
    uint64_t key{0};               // Clave única (SensorID + SampleID) para detectar duplicados rápidamente.
    Time originTime{Seconds(0)};   // Instante exacto de simulación en que el sensor generó el dato original.
    uint32_t sensorId{0};          // Identificador del sensor que produjo la muestra.
    uint32_t sampleId{0};          // Número de secuencia de la muestra generada por ese sensor.
};

// LayerAggregateStats agrega solo las estadísticas de capa que hoy alimentan
// los dashboards: drops por TTL y ocupación media de buffer.
// Sirve para resumir el rendimiento por "roles" (Sensores, Agrobots, UGVs).
struct LayerAggregateStats
{
    uint32_t nodeCount{0};             // Cantidad de nodos contabilizados en esta capa.
    uint64_t ttlDrops{0};              // Total de paquetes descartados porque superaron su Tiempo de Vida (TTL).
    double averageOccupancySum{0.0};   // Sumatoria de las ocupaciones promedio de buffer de todos los nodos de la capa.
};

// SampleHeader es un encabezado de aplicación propio.
// Se eligió heredar de ns3::Header porque la documentación de ns-3 recomienda
// serializar metadatos de protocolo/experimento como headers explícitos cuando
// deben viajar dentro del Packet y recuperarse en nodos intermedios.
//
// Aquí el header preserva identidad de la muestra y su tiempo de origen a través
// de todos los relevos, algo indispensable para delay extremo a extremo y AoI.
class SampleHeader : public Header // Herencia de la clase base Header de ns-3.
{
  public:
    // Tamaño en bytes del header una vez serializado (4+4+8+1 = 17 bytes).
    static constexpr uint32_t SERIALIZED_SIZE = 17;

    SampleHeader(); // Constructor por defecto.

    static TypeId GetTypeId();               // Función estática obligatoria en ns-3 para registrar la clase en el sistema de tipos.
    TypeId GetInstanceTypeId() const override; // Retorna el TypeId de la instancia actual.
    void Print(std::ostream& os) const override; // Define cómo se imprime el header (útil para logs/pcap).
    uint32_t GetSerializedSize() const override; // Retorna cuánto ocupa en memoria al convertirse en bytes.
    void Serialize(Buffer::Iterator start) const override; // Convierte las variables del header a un flujo de bytes en el paquete.
    uint32_t Deserialize(Buffer::Iterator start) override; // Lee el flujo de bytes del paquete y reconstruye las variables.

    // Setters (Modificadores)
    void SetSensorId(uint32_t sensorId);
    void SetSampleId(uint32_t sampleId);
    void SetOriginTimestamp(Time timestamp);
    void SetLogicalHop(uint8_t logicalHop);

    // Getters (Accesorios)
    uint32_t GetSensorId() const;
    uint32_t GetSampleId() const;
    Time GetOriginTimestamp() const;
    
  private:
    uint32_t m_sensorId{0};          // 4 bytes: ID del sensor origen.
    uint32_t m_sampleId{0};          // 4 bytes: Secuencia de la muestra.
    uint64_t m_originTimestampNs{0}; // 8 bytes: Marca de tiempo original en nanosegundos.
    uint8_t m_logicalHop{0};         // 1 byte: Conteo de saltos lógicos (Capa 0, Capa 1, etc.).
};

// Macro de ns-3 que asegura que la clase quede registrada en la base de datos de tipos del simulador al compilar.
NS_OBJECT_ENSURE_REGISTERED(SampleHeader);

// Implementación del constructor por defecto (vacío).
SampleHeader::SampleHeader() = default;

TypeId
SampleHeader::GetTypeId()
{
    // GetTypeId registra la clase en el sistema de tipos de ns-3.
    // Esto permite introspección, creación dinámica y trazabilidad.
    static TypeId tid = TypeId("SampleHeader")
        .SetParent<Header>() // Indica que hereda de la clase base Header.
        .AddConstructor<SampleHeader>(); // Indica cómo instanciar esta clase.
    return tid;
}

TypeId
SampleHeader::GetInstanceTypeId() const
{
    return GetTypeId(); // Simplemente invoca la función estática.
}

void
SampleHeader::Print(std::ostream& os) const
{
    // Print se usa cuando el paquete/header se inspecciona en logs o depuración.
    // Formatea los datos internos hacia el flujo de salida.
    os << "sensor=" << m_sensorId << ", sample=" << m_sampleId
       << ", originNs=" << m_originTimestampNs << ", hop=" << static_cast<uint32_t>(m_logicalHop);
}

uint32_t
SampleHeader::GetSerializedSize() const
{
    return SERIALIZED_SIZE; // Retorna la constante de 17 bytes definida previamente.
}

void
SampleHeader::Serialize(Buffer::Iterator start) const
{
    // Usamos enteros en orden de red para que el contenido del header viaje de
    // forma determinística dentro del Packet y pueda reconstruirse exactamente.
    // 'Hton' significa "Host to Network" (ajusta endianness para asegurar compatibilidad).
    start.WriteHtonU32(m_sensorId);           // Escribe 32 bits (4 bytes).
    start.WriteHtonU32(m_sampleId);           // Escribe 32 bits (4 bytes).
    start.WriteHtonU64(m_originTimestampNs);  // Escribe 64 bits (8 bytes).
    start.WriteU8(m_logicalHop);              // Escribe 8 bits (1 byte).
}

uint32_t
SampleHeader::Deserialize(Buffer::Iterator start)
{
    // Deserialize invierte la serialización y recupera el estado del header
    // cuando un nodo intermedio o la casa recibe una muestra.
    // 'Ntoh' significa "Network to Host". El orden de lectura DEBE ser idéntico al de escritura.
    m_sensorId = start.ReadNtohU32();
    m_sampleId = start.ReadNtohU32();
    m_originTimestampNs = start.ReadNtohU64();
    m_logicalHop = start.ReadU8();
    return SERIALIZED_SIZE; // Retorna cuántos bytes consumió del buffer.
}

// Implementación de Setters y Getters (autoexplicativos).
void SampleHeader::SetSensorId(uint32_t sensorId) { m_sensorId = sensorId; }
void SampleHeader::SetSampleId(uint32_t sampleId) { m_sampleId = sampleId; }
void SampleHeader::SetOriginTimestamp(Time timestamp) { m_originTimestampNs = static_cast<uint64_t>(timestamp.GetNanoSeconds()); }
void SampleHeader::SetLogicalHop(uint8_t logicalHop) { m_logicalHop = logicalHop; }
uint32_t SampleHeader::GetSensorId() const { return m_sensorId; }
uint32_t SampleHeader::GetSampleId() const { return m_sampleId; }
Time SampleHeader::GetOriginTimestamp() const { return NanoSeconds(m_originTimestampNs); } // Convierte el uint64 de vuelta a objeto Time.

// Aplicación principal del modelo de red tolerante a retardos (DTN).
// Implementa el paradigma Store-Carry-Forward (Almacenar-Transportar-Reenviar).
class StoreCarryForwardApp : public Application // Hereda de la clase base Application de ns-3.
{
  public:
    static TypeId GetTypeId(); // Registro en el sistema de tipos.

    StoreCarryForwardApp(); // Constructor.

    // Métodos de configuración llamados desde el script principal (main).
    void SetRoleName(const std::string& roleName); 
    void SetListenPort(uint16_t listenPort);
    void SetNextHop(Ptr<Node> nextHopNode, Ipv4Address nextHopAddress, uint16_t nextHopPort);
    
    // Configura los parámetros clave del algoritmo SCF.
    void SetStoreCarryForwardParameters(uint32_t bufferCapacityPackets,
                                        Time sampleTtl,
                                        Time contactCheckInterval,
                                        uint32_t forwardBurstPackets,
                                        double contactRange);
    void SetForwardHop(uint8_t logicalHop);
    
    // Configura comportamiento de fuente de datos (solo usado por los nodos sensores estáticos).
    void SetSourceParameters(uint32_t sourceId,
                             Time generationInterval,
                             uint32_t packetSize,
                             Time initialGenerationDelay);

    // Métodos para extracción de métricas.
    uint64_t GetTtlDrops() const;
    double GetAverageOccupancy() const;
    uint32_t GetUniqueSourceCount() const;

  protected:
    // Método heredado de Object, llamado al destruir la simulación para limpiar memoria.
    void DoDispose() override;

  private:
    // Métodos heredados de Application que dictan el ciclo de vida.
    void StartApplication() override;
    void StopApplication() override;
    
    // Lógica interna de la aplicación SCF.
    void GenerateSample(); // Crea un nuevo paquete (solo para sensores).
    void CheckContactAndForward(); // Evalúa distancia y envía si hay contacto (Carry -> Forward).
    void HandleRead(Ptr<Socket> socket); // Callback que se ejecuta cuando llega un paquete UDP por red.
    bool EnqueueSample(Ptr<Packet> packet, const SampleHeader& header, bool countAsReceived); // Lógica del "Store" en buffer.
    void PurgeExpiredSamples(); // Borra paquetes cuyo TTL venció.
    bool IsInContact() const; // Calcula si el nodo de siguiente salto está dentro del rango geométrico.
    void UpdateOccupancyIntegral(); // Actualiza la matemática para calcular el promedio real del uso del buffer en el tiempo.
    uint64_t MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const; // Crea un hash/llave única para deduplicación.

    // Variables de estado y parámetros.
    std::string m_roleName{"relay"}; // Etiqueta descriptiva (sensor, agrobot-relay, etc).
    bool m_running{false};           // Bandera para saber si la app está encendida.
    bool m_isSource{false};          // Bandera: ¿Este nodo genera paquetes (sensor) o solo retransmite?
    uint32_t m_sourceId{0};          // ID del sensor si m_isSource es true.
    uint32_t m_nextSampleId{0};      // Contador monótono para numerar las muestras generadas.
    uint32_t m_packetSize{SampleHeader::SERIALIZED_SIZE}; // Tamaño del payload simulado.
    uint16_t m_listenPort{0};        // Puerto UDP para escuchar entrantes.
    uint16_t m_nextHopPort{0};       // Puerto UDP destino del siguiente salto.
    uint8_t m_logicalHop{0};         // Indicador del nivel jerárquico actual.
    uint32_t m_bufferCapacityPackets{128}; // Límite de paquetes en la cola local.
    uint32_t m_forwardBurstPackets{4};     // Máximo de paquetes a vaciar por cada oportunidad de contacto.
    double m_contactRange{30.0};           // Radio geométrico máximo para considerar un "contacto" válido.
    Time m_generationInterval{Seconds(3.0)}; // Frecuencia de generación (solo sensores).
    Time m_initialGenerationDelay{Seconds(0.0)}; // Desfase inicial para no generar todos al mismo tiempo.
    Time m_sampleTtl{Seconds(120.0)};        // Tiempo de vida de la muestra antes de considerarse obsoleta.
    Time m_contactCheckInterval{Seconds(1.0)}; // Frecuencia con la que el nodo "mira" si hay contacto disponible.
    
    // Variables para seguimiento de ocupación del buffer en el tiempo (cálculo integral).
    Time m_activeStart{Seconds(0.0)};
    Time m_lastOccupancyUpdate{Seconds(0.0)};
    Time m_activeDuration{Seconds(0.0)};
    double m_occupancyIntegral{0.0};
    uint64_t m_ttlDrops{0}; // Contador métrico de descartes.
    
    EventId m_generationEvent; // Referencia al evento programado de generación (permite cancelarlo).
    EventId m_contactEvent;    // Referencia al evento programado de chequeo de contacto.
    Ptr<Node> m_nextHopNode;   // Puntero al objeto Nodo que es el destino asignado.
    Ipv4Address m_nextHopAddress; // Dirección IP destino correspondiente a m_nextHopNode.
    Ptr<Socket> m_rxSocket;    // Socket para recepción UDP.
    Ptr<Socket> m_txSocket;    // Socket para transmisión UDP.
    
    std::deque<BufferedSample> m_buffer; // El núcleo del 'Store': la cola FIFO de paquetes.
    std::unordered_set<uint64_t> m_seenSampleKeys; // Tabla hash que recuerda qué paquetes ya pasaron por aquí.
    std::unordered_set<uint32_t> m_uniqueSourceIds; // Tabla hash para contar cuántos sensores distintos se han servido.
};

NS_OBJECT_ENSURE_REGISTERED(StoreCarryForwardApp);

TypeId
StoreCarryForwardApp::GetTypeId()
{
    // La app se registra como ns3::Application para poder instalarse en nodos
    // sin modificar módulos base del simulador, que es la vía correcta para un
    // experimento en scratch/.
    static TypeId tid =
        TypeId("StoreCarryForwardApp")
        .SetParent<Application>()
        .AddConstructor<StoreCarryForwardApp>();
    return tid;
}

StoreCarryForwardApp::StoreCarryForwardApp() = default;

// Setters directos para variables internas.
void StoreCarryForwardApp::SetRoleName(const std::string& roleName) { m_roleName = roleName; }
void StoreCarryForwardApp::SetListenPort(uint16_t listenPort) { m_listenPort = listenPort; }

void
StoreCarryForwardApp::SetNextHop(Ptr<Node> nextHopNode, Ipv4Address nextHopAddress, uint16_t nextHopPort)
{
    // Almacena quién es el jefe superior (siguiente salto lógico y físico).
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
    m_forwardBurstPackets = std::max<uint32_t>(1, forwardBurstPackets); // Evita valores de 0 paquetes por ráfaga.
    m_contactRange = contactRange;
}

void StoreCarryForwardApp::SetForwardHop(uint8_t logicalHop) { m_logicalHop = logicalHop; }

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

// Getters para métricas estadísticas extraídas al final del experimento.
uint64_t StoreCarryForwardApp::GetTtlDrops() const { return m_ttlDrops; }

double
StoreCarryForwardApp::GetAverageOccupancy() const
{
    // Cálculo integral del promedio matemático a lo largo del tiempo de la simulación.
    if (m_activeDuration.IsZero())
    {
        return 0.0;
    }
    return m_occupancyIntegral / m_activeDuration.GetSeconds();
}

uint32_t StoreCarryForwardApp::GetUniqueSourceCount() const { return m_uniqueSourceIds.size(); }

void
StoreCarryForwardApp::DoDispose()
{
    // DoDispose libera referencias ns-3/Ptr y limpia estructuras auxiliares.
    // Esto evita que queden sockets o punteros vivos tras Simulator::Destroy() 
    // previniendo fugas de memoria (memory leaks) en ns-3.
    m_buffer.clear();
    m_seenSampleKeys.clear();
    m_uniqueSourceIds.clear();
    m_nextHopNode = nullptr;
    m_rxSocket = nullptr;
    m_txSocket = nullptr;
    Application::DoDispose(); // Llama al Dispose de la clase padre.
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
        // y evita agregar complejidad de transporte confiable (como TCP) que rompería SCF.
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_rxSocket = Socket::CreateSocket(GetNode(), tid);
        // Bind ancla el socket al puerto para empezar a escuchar en todas las interfaces (GetAny).
        if (m_rxSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_listenPort)) == -1)
        {
            NS_FATAL_ERROR("No se pudo hacer bind del socket UDP para " << m_roleName);
        }
        // Asigna el callback: cuando llegue un paquete, ejecuta el método HandleRead.
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
        // Connect asocia permanentemente la IP de destino a este socket para facilitar el Send().
        if (m_txSocket->Connect(InetSocketAddress(m_nextHopAddress, m_nextHopPort)) == -1)
        {
            NS_FATAL_ERROR("No se pudo conectar el socket emisor UDP para " << m_roleName);
        }
    }

    if (m_isSource)
    {
        // Si el nodo es sensor, agenda el primer evento de creación de muestra en el futuro.
        m_generationEvent =
            Simulator::Schedule(m_initialGenerationDelay, &StoreCarryForwardApp::GenerateSample, this);
    }

    if (m_nextHopNode && m_nextHopPort != 0)
    {
        // Inicia el ciclo infinito de revisar si estamos cerca del objetivo (a los 0.1s empieza).
        m_contactEvent =
            Simulator::Schedule(Seconds(0.1), &StoreCarryForwardApp::CheckContactAndForward, this);
    }
}

void
StoreCarryForwardApp::StopApplication()
{
    // Apaga de manera limpia la aplicación.
    m_running = false;
    
    // Cancela eventos programados en el calendario de ns-3 que aún no se ejecutaron.
    if (m_generationEvent.IsPending())
    {
        Simulator::Cancel(m_generationEvent);
    }
    if (m_contactEvent.IsPending())
    {
        Simulator::Cancel(m_contactEvent);
    }

    // Cierra contadores de métricas.
    UpdateOccupancyIntegral();
    m_activeDuration = Simulator::Now() - m_activeStart;

    // Desconecta callbacks y cierra la capa de transporte (sockets).
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

    PurgeExpiredSamples(); // Limpia la basura antes de crear algo nuevo para liberar espacio.

    SampleHeader header;
    // El header identifica unívocamente la medición y fija su timestamp origen.
    header.SetSensorId(m_sourceId);
    header.SetSampleId(m_nextSampleId++); // Incrementa el identificador global de este sensor.
    header.SetOriginTimestamp(Simulator::Now()); // Marca de tiempo fundamental para el AoI final.
    header.SetLogicalHop(m_logicalHop);

    // Calcula de qué tamaño debe ser la carga de bytes inútiles (payload padding) 
    // para cumplir con el 'packetSize' estipulado por CLI.
    const uint32_t payloadSize =
        (m_packetSize > SampleHeader::SERIALIZED_SIZE) ? (m_packetSize - SampleHeader::SERIALIZED_SIZE)
                                                       : 0;
    // Creamos payload "vacío" del tamaño requerido y luego añadimos el header.
    // En este experimento importa más el transporte de metadatos que el contenido.
    Ptr<Packet> packet = Create<Packet>(payloadSize);
    packet->AddHeader(header);

    EnqueueSample(packet, header, false); // Guarda el paquete generado en el propio buffer.

    // Programa a futuro su propia re-ejecución (bucle iterativo de generación periódica).
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

    PurgeExpiredSamples(); // Regla de oro DTN: antes de enviar, tirar lo vencido.

    // Si hay datos, hay a quien enviarle, el socket existe, Y ESTAMOS A RANGO GEOMÉTRICO (Contacto Físico).
    if (!m_buffer.empty() && m_nextHopNode && m_txSocket && IsInContact())
    {
        uint32_t forwardedThisRound = 0;
        // Bucle de vaciado de buffer (Limitado por 'forwardBurstPackets' para no saturar el canal WiFi de golpe).
        while (forwardedThisRound < m_forwardBurstPackets && !m_buffer.empty())
        {
            const BufferedSample record = m_buffer.front(); // Saca de la punta de la cola (FIFO).
            SampleHeader header;
            Ptr<Packet> inspect = record.packet->Copy(); // Copia del paquete en RAM.
            inspect->RemoveHeader(header); // Saca el header para poder modificarlo.
            
            // Reescribimos el logicalHop para reflejar desde qué capa sale ahora
            // la muestra, pero preservamos sensorId/sampleId/originTimestamp intactos.
            header.SetLogicalHop(m_logicalHop);
            
            // Reconstruye el tamaño original.
            const uint32_t payloadSize =
                (record.packet->GetSize() > SampleHeader::SERIALIZED_SIZE)
                    ? (record.packet->GetSize() - SampleHeader::SERIALIZED_SIZE)
                    : 0;
            Ptr<Packet> outbound = Create<Packet>(payloadSize);
            outbound->AddHeader(header); // Añade el header actualizado.

            // Intenta inyectar a la capa IP/MAC.
            if (m_txSocket->Send(outbound) < 0)
            {
                // Si el socket no acepta el envío (ej. buffer ARP lleno, o de red caído),
                // se conserva el resto del buffer para próximos contactos; no se destruye el estado SCF.
                break; // Rompe el loop de burst, volverá a intentar en el próximo check.
            }

            // Operación exitosa: contabiliza la matemática del buffer y saca el elemento.
            UpdateOccupancyIntegral();
            m_buffer.pop_front();

            ++forwardedThisRound;
        }
    }

    // Reprograma de forma recursiva la función a sí misma, manteniendo el reloj del SCF vivo.
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
    Address from; // Guardará la info de quién lo envió.
    // Loop de lectura exhaustiva: lee hasta que el socket diga que no hay más.
    while ((packet = socket->RecvFrom(from)))
    {
        Ptr<Packet> inspect = packet->Copy();
        SampleHeader header;
        inspect->RemoveHeader(header); // Extrae la metadata inyectada en origen.
        // Lo pasa a la función de encolado para decidir si se guarda o se bota (si está lleno o repetido).
        EnqueueSample(packet, header, true); 
    }
}

bool
StoreCarryForwardApp::EnqueueSample(Ptr<Packet> packet, const SampleHeader& header, bool countAsReceived)
{
    // Cada muestra se indexa por (sensorId, sampleId). Esa clave evita contar
    // duplicados múltiples veces en relays y mantiene trazabilidad por muestra.
    const uint64_t key = MakeSampleKey(header.GetSensorId(), header.GetSampleId());

    // Regla Antiduplicados de Epidemic/SCF:
    if (countAsReceived)
    {
        // Si el hash ya existe en mi memoria histórica, tiro el paquete y retorno false.
        if (m_seenSampleKeys.find(key) != m_seenSampleKeys.end())
        {
            return false;
        }
        m_seenSampleKeys.insert(key); // Registro el hash como "ya visto".
        m_uniqueSourceIds.insert(header.GetSensorId()); // Registro que cubrí este sensor.
    }

    // Regla de Capacidad (Control de Congestión DTN):
    if (m_buffer.size() >= m_bufferCapacityPackets)
    {
        // Un overflow representa una limitación local del relay, no del canal.
        // Pierdo el paquete silenciósamente (drop clásico de cola).
        return false;
    }

    // El paquete es válido, nuevo, y hay espacio.
    UpdateOccupancyIntegral();

    // Empaqueta metadata local para la vida del buffer.
    BufferedSample sample;
    sample.packet = packet->Copy();
    sample.key = key;
    sample.originTime = header.GetOriginTimestamp(); // Super importante para el TTL.
    sample.sensorId = header.GetSensorId();
    sample.sampleId = header.GetSampleId();
    
    // Inserción en la cola FIFO local del dron/UGV.
    m_buffer.push_back(sample);

    return true;
}

void
StoreCarryForwardApp::PurgeExpiredSamples()
{
    // TTL se evalúa contra originTime y no contra enqueueTime porque la edad
    // relevante es la de la información desde que nació en el sensor.
    auto it = m_buffer.begin();
    while (it != m_buffer.end()) // Itera sobre toda la cola
    {
        // Si (Ahora - Nacimiento) > Tolerancia...
        if ((Simulator::Now() - it->originTime) > m_sampleTtl)
        {
            UpdateOccupancyIntegral();
            it = m_buffer.erase(it); // Lo elimina de memoria.
            ++m_ttlDrops; // Contabiliza como dato obsoleto/muerto.
        }
        else
        {
            ++it; // Avanza al siguiente elemento de la cola de forma segura.
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

    // Obtenemos los modelos de movilidad tridimensionales atados a mi nodo y al objetivo.
    Ptr<MobilityModel> selfMobility = GetNode()->GetObject<MobilityModel>();
    Ptr<MobilityModel> peerMobility = m_nextHopNode->GetObject<MobilityModel>();

    if (!selfMobility || !peerMobility)
    {
        return false;
    }

    // Calcula distancia Euclideana usando utilidades matemáticas de ns-3.
    const double distance =
        CalculateDistance(selfMobility->GetPosition(), peerMobility->GetPosition());
        
    // Si la distancia física es menor al radio configurado, declaro que HAY CONTACTO SCF.
    return distance <= m_contactRange;
}

void
StoreCarryForwardApp::UpdateOccupancyIntegral()
{
    // Integramos ocupación en el tiempo para obtener promedio temporal real
    // del buffer, no solo snapshots puntuales (Técnica de Integral de Riemann discreta).
    const double deltaSeconds = (Simulator::Now() - m_lastOccupancyUpdate).GetSeconds();
    m_occupancyIntegral += deltaSeconds * static_cast<double>(m_buffer.size());
    m_lastOccupancyUpdate = Simulator::Now();
}

uint64_t
StoreCarryForwardApp::MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const
{
    // Función Hash bit a bit: desplaza 32 bits a la izq el SensorID y hace un OR (suma binaria) 
    // con el SampleID para crear un uint64_t único y rápido de buscar.
    return (static_cast<uint64_t>(sensorId) << 32) | static_cast<uint64_t>(sampleId);
}


// Definición de la Aplicación del Nodo Destino Central (La Casa / Gateway final)
class HouseCollectorApp : public Application // Hereda de ns3::Application
{
  public:
    static TypeId GetTypeId(); // Registro en la base de datos de objetos ns-3.

    HouseCollectorApp();

    // Setup principal
    void Configure(uint16_t listenPort, uint32_t sensorCount, Time aoiSamplePeriod);

    // Getters de métricas globales del experimento
    uint32_t GetSensorsDeliveredToHouse() const;
    double GetAverageEndToEndDelay() const;
    double GetAverageAoi() const;
    double GetCurrentAverageAoi() const;

  protected:
    void DoDispose() override; // Limpiador de memoria.

  private:
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket); // Callback de recepción UDP.
    void SampleAoi(); // Disparador periódico para guardar el estado del Age of Information.
    void RecordAoiSnapshot(); // Ejecución lógica matemática de fotografiar el AoI.
    uint64_t MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const;

    // Variables de estado
    bool m_running{false};
    uint16_t m_listenPort{0};
    uint32_t m_sensorCount{0};
    Time m_aoiSamplePeriod{Seconds(1.0)};
    Ptr<Socket> m_rxSocket;
    EventId m_aoiEvent;
    
    std::unordered_set<uint64_t> m_deliveredKeys; // Hashset global de muestras que entraron a la casa.
    std::vector<bool> m_sensorDelivered; // Vector booleano: true si el sensor X ya reportó algo.
    std::vector<bool> m_hasFreshUpdate; // Vector booleano: true si el sensor X tiene al menos 1 reporte activo.
    std::vector<Time> m_latestOriginTimestamp; // Array que guarda la "edad de la foto más reciente" por sensor.
    
    // Acumuladores de promedios para métricas al final de la corrida.
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
    // Dimensionamos arrays para mapear estado a nivel de cada sensor indexado.
    m_sensorDelivered.assign(sensorCount, false);
    m_hasFreshUpdate.assign(sensorCount, false);
    m_latestOriginTimestamp.assign(sensorCount, Seconds(0.0));
}

uint32_t
HouseCollectorApp::GetSensorsDeliveredToHouse() const
{
    // std::count cuenta cuántos 'true' hay en el vector. Representa la métrica 'Coverage'.
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
    // Esta función saca un promedio instantáneo del estado actual para la serie de tiempo.
    double sumAoi = 0.0;
    uint32_t activeSensors = 0;

    for (uint32_t i = 0; i < m_sensorCount; ++i)
    {
        // Solo evaluamos sensores que ya han establecido al menos un enlace exitoso.
        if (!m_hasFreshUpdate[i])
        {
            continue;
        }
        // Formula clásica AoI: (Tiempo Actual - Tiempo del Último dato fresco)
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
    // Limpieza agresiva de vectores para gestión de memoria.
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

    // Activa el muestreo infinito del dashboard de Age of Information.
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
    while ((packet = socket->RecvFrom(from))) // Lee todos los datagramas pendientes.
    {
        Ptr<Packet> inspect = packet->Copy();
        SampleHeader header;
        inspect->RemoveHeader(header);

        // Deduplicación Global a Nivel de Experimento:
        const uint64_t key = MakeSampleKey(header.GetSensorId(), header.GetSampleId());
        if (m_deliveredKeys.find(key) != m_deliveredKeys.end())
        {
            continue; // Ya vi este datagrama viajando por otro dron, lo ignoro.
        }
        m_deliveredKeys.insert(key);

        // Dimension E2E: delay extremo a extremo calculado mágicamente 
        // usando relojes sincronizados del simulador.
        const double delaySeconds = (Simulator::Now() - header.GetOriginTimestamp()).GetSeconds();
        // Delay e2e se calcula con el timestamp origen preservado en SampleHeader.
        m_delaySumSeconds += delaySeconds;
        ++m_delaySamples;

        // Actualización de estado del origen (Sensor Coverage y AoI Update)
        if (header.GetSensorId() < m_sensorCount)
        {
            m_sensorDelivered[header.GetSensorId()] = true; // Activa flag de cobertura.
            
            // Si es la primera actualización O la actualización nueva fue generada DESPUÉS de la que ya tenía...
            // (Evita que paquetes atrasados que tomaron una ruta larga via un nodo UGV lento dañen la frescura)
            if (!m_hasFreshUpdate[header.GetSensorId()] ||
                header.GetOriginTimestamp() > m_latestOriginTimestamp[header.GetSensorId()])
            {
                m_hasFreshUpdate[header.GetSensorId()] = true;
                // Renueva la estampa de tiempo más fresca de la tabla.
                m_latestOriginTimestamp[header.GetSensorId()] = header.GetOriginTimestamp();
            }
        }
    }

    RecordAoiSnapshot(); // Forzamos un registro estadístico inmediato por la llegada del paquete.
}

void
HouseCollectorApp::SampleAoi()
{
    // AoI también se muestrea periódicamente porque interesa su evolución en el
    // tiempo, no solo el valor en instantes de recepción. (El AoI sube como rampa
    // lineal con el tiempo cuando NO hay paquetes nuevos).
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
            continue; // Skips dead/unreachable sensors to avoid polluting metrics with infinities.
        }

        const double ageSeconds = (Simulator::Now() - m_latestOriginTimestamp[i]).GetSeconds();
        sumAoi += ageSeconds;
        ++activeSensors;
    }

    if (activeSensors > 0)
    {
        m_aoiSnapshotSumSeconds += sumAoi / static_cast<double>(activeSensors);
        ++m_aoiSnapshotSamples; // Contador de integral discreta.
    }
}

uint64_t
HouseCollectorApp::MakeSampleKey(uint32_t sensorId, uint32_t sampleId) const
{
    // Función Hash bit a bit idéntica a la capa SCF.
    return (static_cast<uint64_t>(sensorId) << 32) | static_cast<uint64_t>(sampleId);
}


// --- BLOQUE DE FUNCIONES AUXILIARES GLOBALES (ESTÁTICAS) ---

static std::string
MakeUniformRv(double minValue, double maxValue)
{
    // ns-3 acepta variables aleatorias como StringValue en muchos helpers.
    // Esta utilidad evita repetir cadenas largas y mantiene parámetros claros.
    // Ej: transforma Min=0, Max=100 a "ns3::UniformRandomVariable[Min=0|Max=100]".
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
    // Asigna el generador de Random Variables formateado string en ejes X y Y.
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
    // En este experimento sirve para recorridos repetibles de líderes (Patrón Cortadora de Césped para los UGVs).
    const double speed = std::max(0.5, leaderSpeed); // Garantiza velocidad no nula.

    if (horizontalSweep)
    {
        // Barrido horizontal (izq-der, der-izq) con márgenes de 5 metros de los bordes.
        const double xMin = fieldBounds.xMin + 5.0;
        const double xMax = fieldBounds.xMax - 5.0;
        const double legDuration = std::max(1.0, (xMax - xMin) / speed); // t = d/v
        const double laneSpacing =
            (fieldBounds.yMax - fieldBounds.yMin) / static_cast<double>(leaders.GetN() + 1);

        for (uint32_t i = 0; i < leaders.GetN(); ++i)
        {
            Ptr<WaypointMobilityModel> waypoint = leaders.Get(i)->GetObject<WaypointMobilityModel>();
            if (!waypoint)
            {
                NS_FATAL_ERROR("WaypointMobilityModel no instalado en lider horizontal " << i);
            }

            // Reparte los líderes en carriles Y paralelos.
            const double laneY = fieldBounds.yMin + laneSpacing * static_cast<double>(i + 1);
            double t = 0.0;
            double x = xMin;

            waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
            // Cada waypoint fija una trayectoria de barrido por carriles.
            while (t + legDuration <= simTime + legDuration) // Repite hasta agotar simTime
            {
                t += legDuration;
                x = (x == xMin) ? xMax : xMin; // Alterna bordes (efecto rebote lateral)
                waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
            }
        }
        return;
    }

    // Elipse del barrido vertical (mismo concepto, diferente eje principal). Usado en default para UGVs.
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
    // respecto de ese líder, así que el cluster sigue existiendo como grupo (Patrón Swarm para Drones Agrobot).
    Ptr<RandomRectanglePositionAllocator> initialPositions =
        CreateUniformPositionAllocator(fieldBounds);
    Ptr<RandomRectanglePositionAllocator> destinationPositions =
        CreateUniformPositionAllocator(fieldBounds);

    // Instala el helper conectando el modelo con los variables aleatorias generadas para velocidad y pausa.
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
    // y "movimiento de nodo" al mismo tiempo. (El seguidor "órbita" en RandomWalk2D alrededor del líder).
    GroupMobilityHelper mobility;
    mobility.SetReferenceMobilityModel(leaderMobility); // Modela el centro de masas dictado por el líder.
    mobility.SetMemberPositionAllocator(CreateUniformPositionAllocator(localBounds)); // Offset geométrico inicial de seguidores dentro del enjambre.
    mobility.SetMemberMobilityModel("ns3::RandomWalk2dMobilityModel",
                                    "Bounds",
                                    RectangleValue(localBounds), // Los seguidores no pueden alejarse fuera de esta caja virtual.
                                    "Mode",
                                    EnumValue(RandomWalk2dMobilityModel::MODE_TIME),
                                    "Time",
                                    TimeValue(Seconds(2.0)), // Cambian dirección cada 2 segundos.
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
    // Se usa para nodos cuya posición debe fijarse explícitamente, como la casa (x,y,z manuales quemados en código).
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
    // Esta utilidad implementa asignación por proximidad geométrica estática (solo sirve en T=0 o posiciones estáticas).
    // Se usa tanto para sensor->agrobot como para emparejar agrobots con UGVs.
    uint32_t bestIndex = 0;
    double bestDistance2 = std::numeric_limits<double>::max(); // Comienza en infinito.

    for (uint32_t i = 0; i < candidates.GetN(); ++i)
    {
        const Vector candidatePosition = candidates.Get(i)->GetObject<MobilityModel>()->GetPosition();
        // Distancia euclideana al cuadrado (evita uso intensivo de CPU calculando raiz cuadrada).
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
    // Esto previene que un solo UGV se ahogue bajo el tráfico de todos los Agrobots de la granja.
    const uint32_t invalidAssignment = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> assignment(agrobotLeaders.GetN(), invalidAssignment);
    std::vector<bool> agrobotTaken(agrobotLeaders.GetN(), false);

    // Bucle para fase "greedy" 1-a-1
    for (uint32_t ugvIndex = 0; ugvIndex < ugvLeaders.GetN(); ++ugvIndex)
    {
        const Vector ugvPosition = ugvLeaders.Get(ugvIndex)->GetObject<MobilityModel>()->GetPosition();
        uint32_t bestAgrobotIndex = invalidAssignment;
        double bestDistance2 = std::numeric_limits<double>::max();

        for (uint32_t agrobotIndex = 0; agrobotIndex < agrobotLeaders.GetN(); ++agrobotIndex)
        {
            if (agrobotTaken[agrobotIndex])
            {
                continue; // Skips ya asignados.
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

    // Bucle para fase de "sobras"
    for (uint32_t agrobotIndex = 0; agrobotIndex < agrobotLeaders.GetN(); ++agrobotIndex)
    {
        if (assignment[agrobotIndex] != invalidAssignment)
        {
            continue; // Este ya fue procesado con éxito.
        }

        const Vector agrobotPosition =
            agrobotLeaders.Get(agrobotIndex)->GetObject<MobilityModel>()->GetPosition();
        assignment[agrobotIndex] = FindNearestNode(agrobotPosition, ugvLeaders); // Lo manda al global más cercano.
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
    // destino para reconstruir las tres etapas lógicas del experimento y extraer info general IP.
    StageMetrics metrics;

    for (const auto& [flowId, st] : stats)
    {
        const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(flowId);
        // Regla: ¿Este tráfico pertenece a la etapa/flujo lógico que estoy buscando?
        const bool isUdpDataFlow = (tuple.protocol == 17) && (tuple.destinationPort == destinationPort);
        if (!isUdpDataFlow)
        {
            continue;
        }

        metrics.txPackets += st.txPackets;
        metrics.rxPackets += st.rxPackets;
        metrics.totalDelaySeconds += st.delaySum.GetSeconds();
        metrics.rxDelaySamples += st.rxPackets;

        // Búsqueda en mapeo para verificar si un origen específico IP contribuyó exitosamente.
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
    // y en el CSV final sin duplicar código. (Evita división por cero).
    if (metrics.txPackets == 0)
    {
        return 0.0;
    }

    return 100.0 * static_cast<double>(metrics.rxPackets) / static_cast<double>(metrics.txPackets);
}

static LayerAggregateStats
AggregateLayerStats(const std::vector<Ptr<StoreCarryForwardApp>>& apps)
{
    // Reduce varias apps homogéneas (ej. Todos los Agrobots) a la vista mínima de capa usada por el
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
    // Promedio matemático inter-nodo de las variables de memoria ocupada en RAM (buffers).
    if (stats.nodeCount == 0)
    {
        return 0.0;
    }
    return stats.averageOccupancySum / static_cast<double>(stats.nodeCount);
}

static bool
FileHasContent(const std::string& fileName)
{
    // Sirve para no reescribir encabezados CSV cuando una campaña de barrido de Python
    // acumula varias corridas en el mismo archivo (Evita romper Pandas en post-análisis).
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
    // Todas las exportaciones se hacen por append (modo concatenar) para no sobrescribir artefactos
    // previos sin instrucción explícita del usuario.
    std::ofstream output(fileName, std::ios::app);
    output << line << "\n";
}

static std::string
FormatDouble(double value, uint32_t precision = 6)
{
    // Formato estable para CSV: evita notación científica nativa de C++ y mantiene columnas
    // consistentes al postprocesar en Python, R o Excel.
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

static std::string
JoinCsvFields(const std::vector<std::string>& fields)
{
    // Ensambla una fila CSV simple inyectando comas. No escapamos comas internamente porque los campos que
    // exporta este experimento son numéricos rígidos o etiquetas sin separadores inyectados.
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
    // Etiqueta por defecto para distinguir corridas sin exigir un nombre manual desde bash/python.
    // Se usa tanto en CSV resumen como en serie temporal.
    std::ostringstream oss;
    oss << labelPrefix << "-run" << rngRun;
    return oss.str();
}

static uint32_t
CountCoveredSensorsAtAgrobotLevel(const std::vector<Ptr<StoreCarryForwardApp>>& agrobotRelayApps)
{
    // Agrega la cobertura del primer nivel sumando las fuentes únicas vistas por
    // cada relay agrobot. Es una cobertura "hasta nivel 1", distinta de la entregada a la casa en nivel 3.
    uint32_t coveredSensors = 0;
    for (const auto& app : agrobotRelayApps)
    {
        coveredSensors += app->GetUniqueSourceCount();
    }
    return coveredSensors;
}

} // namespace anónimo (protección interna para que estas funciones no escapen de este fichero).

// Función Main: Punto de inyección del núcleo C++ hacia ns-3
int
main(int argc, char* argv[])
{
    // Reglas mínimas de validez que protegen a la simulación de configuraciones
    // imposibles o inconsistentes con la API de ns-3.
    const uint32_t minClusterNodes = 2; // Un cluster requiere mínimo 1 lider + 1 seguidor.
    const uint32_t nLevel1Clusters = 3; // Harcoded a 3 flotillas drones.
    const uint32_t nLevel2Clusters = 2; // Hardcoded a 2 tractores terrestres UGVs.

    // -------------------------------------------------------------------------
    // Parametros de experimento por Default (Si CLI no interviene)
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

    // Baseline geométrico y de movilidad sobre el que se montan las campañas.
    // Cambiar estos valores modifica la cinemática del experimento, no solo la
    // forma de observarlo.
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
    uint32_t rngRun = 1; // Semilla de aleatoriedad
    std::string flowmonFile = "flowmon-adhoc.xml"; // Nombre base de dump logico

    // Parámetros que sí cambian la semántica store-carry-forward: oportunidad
    // de contacto, persistencia de la información y presión sobre la memoria local.
    double sensorContactRange = 35.0; // Distancia (radio RF teorico) asimilada a contacto de software.
    double level1ToLevel2ContactRange = 30.0;
    double level2ToHouseContactRange = 50.0;
    double contactCheckInterval = 1.0;
    uint32_t forwardBurstPackets = 6;
    uint32_t bufferCapacityPackets = 128;
    double sampleTtl = 120.0; // La data agrícola deja de ser valiosa a los 2 minutos.
    double aoiSamplePeriod = 1.0;
    double metricsSampleInterval = 1.0; // Solo cambia la resolución de exportación, no la dinámica de red.
    std::string summaryCsvFile = "summary-adhoc.csv";
    std::string timeseriesCsvFile = "timeseries-adhoc.csv";
    std::string metricsRunLabel;

    // CommandLine expone todos los parámetros experimentales como comandos de Terminal CLI (ej: ./ns3 run "adhoc --simTime=200").
    // Esto sigue la práctica habitual en ejemplos de ns-3: una sola simulación
    // reutilizable con distintos valores sin recompilar C++.
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
                 "Velocidad maxima de seguidores agrobot (m/s); afecta la movilidad local del cluster, no toda la red",
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
                 "Umbral de contacto sensor->lider agrobot para store-carry-forward (m); aproxima la oportunidad del primer relevo",
                 sensorContactRange);
    cmd.AddValue("level1ToLevel2ContactRange",
                 "Umbral de contacto lider agrobot -> lider UGV para store-carry-forward (m); parametro barrido en contact_range_campaign",
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
    cmd.AddValue("bufferCapacityPackets",
                 "Capacidad de buffer por nodo store-carry-forward; controla saturacion local de relays",
                 bufferCapacityPackets);
    cmd.AddValue("sampleTtl",
                 "TTL de cada muestra en store-carry-forward (s); modela cuanto tiempo sigue siendo util la informacion",
                 sampleTtl);
    cmd.AddValue("aoiSamplePeriod", "Periodo de muestreo de Age of Information en la casa (s)", aoiSamplePeriod);
    cmd.AddValue("metricsSampleInterval",
                 "Periodo de muestreo para exportar la serie temporal de metricas (s); no cambia forwarding ni movilidad",
                 metricsSampleInterval);
    cmd.AddValue("summaryCsvFile", "Archivo CSV de resumen final de la corrida", summaryCsvFile);
    cmd.AddValue("timeseriesCsvFile", "Archivo CSV de serie temporal de metricas", timeseriesCsvFile);
    cmd.AddValue("metricsRunLabel", "Etiqueta de corrida para exportacion CSV", metricsRunLabel);
    
    // Procesa e inyecta los flags que el usuario escribió desde consola (sobrescribiendo los defaults).
    cmd.Parse(argc, argv);

    // Mínimo logístico de bytes de paquete para no crear un datagrama truncado o malformado en ns-3.
    const uint32_t minScfPacketSize = SampleHeader::SERIALIZED_SIZE;
    if (metricsRunLabel.empty())
    {
        metricsRunLabel = BuildDefaultRunLabel("scf", rngRun);
    }

    // Validación temprana para fallar con un mensaje claro antes de crear nodos,
    // sockets o eventos. Esto ayuda mucho al depurar campañas de barrido que 
    // duran horas e impiden que una config mala crashee a la mitad.
    if (agrobotsPerCluster < minClusterNodes || level2NodesPerCluster < minClusterNodes ||
        sensorsPerField < 1 || simTime <= 5.0 || fieldWidth <= 20.0 || fieldHeight <= 20.0 ||
        clusterBoxWidth <= 1.0 || clusterBoxHeight <= 1.0 || level2BoxWidth <= 1.0 ||
        level2BoxHeight <= 1.0 || sensorInterval <= 0.0 || contactCheckInterval <= 0.0 ||
        forwardBurstPackets < 1 || bufferCapacityPackets < 1 || sampleTtl <= 0.0 ||
        aoiSamplePeriod <= 0.0 || metricsSampleInterval <= 0.0)
    {
        std::cerr << "Parametros invalidos para el experimento." << std::endl;
        return 1; // Salida con código de Error de C++.
    }

    // Nuestro SampleHeader debe caber completo dentro del paquete de cada muestra.
    if (packetSize < minScfPacketSize)
    {
        std::cerr << "packetSize debe ser >= " << minScfPacketSize
                  << " bytes para alojar SampleHeader." << std::endl;
        return 1;
    }

    // Validación de pausa de waypoint física imposible en tiempo negativo.
    if (agrobotLeaderPause < 0.0)
    {
        std::cerr << "agrobotLeaderPause no puede ser negativo." << std::endl;
        return 1;
    }

    // RngSeedManager::SetRun cambia la subsecuencia aleatoria manteniendo la semilla
    // global. Es la forma estándar de ns-3 para generar réplicas reproducibles (Monter Carlo).
    RngSeedManager::SetRun(rngRun);

    // Derivamos tamaños totales y geometría del escenario a partir de la CLI (para iteraciones automáticas).
    const uint32_t agrobotFollowersPerCluster = agrobotsPerCluster - 1; // Le quitamos el lider.
    const uint32_t ugvFollowersPerCluster = level2NodesPerCluster - 1;
    const uint32_t totalAgrobotNodes = nLevel1Clusters * agrobotsPerCluster;
    const uint32_t totalUgvNodes = nLevel2Clusters * level2NodesPerCluster;
    const Rectangle fieldBounds(0.0, fieldWidth, 0.0, fieldHeight); // Mapa geográfico (Limites XY)
    // Limites virtuales para agrupar enjambre Agrobot (Centroide de 0.0 relativo al lider)
    const Rectangle agrobotLocalBounds(-clusterBoxWidth * 0.5,
                                       clusterBoxWidth * 0.5,
                                       -clusterBoxHeight * 0.5,
                                       clusterBoxHeight * 0.5);
    // Limites virtuales para agrupar UGVs                      
    const Rectangle ugvLocalBounds(-level2BoxWidth * 0.5,
                                   level2BoxWidth * 0.5,
                                   -level2BoxHeight * 0.5,
                                   level2BoxHeight * 0.5);
    // Posición estática e inmutable de la antena de la casa agrícola.
    const Vector housePosition(fieldWidth * 0.5, fieldHeight * 0.5, 0.0);

    // Cada conjunto de Nodos C++ (NodeContainer) se crea por separado para poder asignarle movilidad, potencia
    // y aplicaciones distintas según su rol técnico en la jerarquía estructurada.
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
    // base de ns-3 para instalar helpers (Protocolos, IPs, HW) sobre subconjuntos del escenario
    // en una sola instrucción elegante, en lugar de loops de `for`.
    NodeContainer agrobotNodes;
    agrobotNodes.Add(agrobotLeaders);
    agrobotNodes.Add(agrobotFollowers);

    NodeContainer ugvNodes;
    ugvNodes.Add(ugvLeaders);
    ugvNodes.Add(ugvFollowers);

    NodeContainer allNodes; // Contenedor Dios: Contiene absolutamente toda la topología.
    allNodes.Add(agrobotNodes);
    allNodes.Add(ugvNodes);
    allNodes.Add(sensors);
    allNodes.Add(house);

    // Reconstruimos explícitamente la partición por cluster para instalar
    // movilidad relativa por grupo de la arquitectura de la red (Swarm Theory).
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

    // Configuración 1 a 1 de enjambre de robots voladores atado a su propio Lider de cuadrilla.
    for (uint32_t c = 0; c < nLevel1Clusters; ++c)
    {
        InstallRelativeClusterMobility(agrobotClusterFollowers[c],
                                       agrobotLeaders.Get(c)->GetObject<MobilityModel>(),
                                       agrobotLocalBounds,
                                       followerSpeedMax);
    }

    // Instalación de Cortadora de Cesped en Lideres Terrestres UGVs.
    MobilityHelper ugvLeaderMobility;
    ugvLeaderMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    ugvLeaderMobility.Install(ugvLeaders);
    ConfigureSweepWaypoints(ugvLeaders, simTime, fieldBounds, level2LeaderSpeed, false);

    // Seguidores de UGVs atados al tractor principal.
    for (uint32_t c = 0; c < nLevel2Clusters; ++c)
    {
        InstallRelativeClusterMobility(ugvClusterFollowers[c],
                                       ugvLeaders.Get(c)->GetObject<MobilityModel>(),
                                       ugvLocalBounds,
                                       level2FollowerSpeedMax);
    }

    // Nodos Fijos (Sensores de suelo enterrados / Casa de comando).
    InstallSensorMobility(sensors, fieldBounds);
    InstallFixedMobility(house, {housePosition});

    // -------------------------------------------------------------------------
    // WiFi ad-hoc + AODV
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // CAPA FÍSICA Y MAC (Data Link)
    // -------------------------------------------------------------------------
    
    // WifiHelper::SetStandard fija el estándar 802.11g, que en ns-3 configura
    // banda de 2.4 GHz y ancho de canal de 20 MHz, suficiente para una MANET
    // simple sin la complejidad extra (y peso computacional) de 802.11n/ac MIMO.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    
    // ConstantRateWifiManager fuerza tasas fijas de datos/control.
    // Se eligió para que los cambios observados en la investigación provengan más de la movilidad y algoritmos SCF
    // que de la adaptación algorítmica caótica que implementa el ARF Rate Adaptation internamente en 802.11.
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate12Mbps"), // Transmisión Datos
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps")); // Transmisión RTS/CTS y ACK.

    WifiMacHelper mac;
    // AdhocWifiMac construye una red sin Access Points (AP) ni infraestructura, coherente con la
    // hipótesis MANET de malla flotante y de libre asociación del taller de arquitectura.
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiChannelHelper channelHelper;
    // El retardo de propagación a velocidad constante (Velocidad de la luz c) es la opción estándar del
    // helper Yans (Yet Another Network Simulator) para modelar la propagación de ondas RF sobre un canal inalámbrico simple.
    channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    
    // LogDistancePropagationLossModel con ReferenceLoss=40.046 calibra la pérdida
    // de referencia en el primer metro a 2.4 GHz, alineada con 802.11g según los papers técnicos y ejemplos canónicos de ns-3.
    channelHelper.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                     "ReferenceLoss",
                                     DoubleValue(40.046));
    Ptr<YansWifiChannel> wifiChannel = channelHelper.Create(); // Instancia el canal electromagnético para la simulación.

    // Usamos PHY distintos por rol solo para fijar potencias TX (Transmisión RF) diferentes sin
    // crear canales separados lógicamente: todos comparten el mismo medio inalámbrico global de colisión electromagnética.
    YansWifiPhyHelper agrobotPhy;
    agrobotPhy.SetChannel(wifiChannel);
    agrobotPhy.Set("TxPowerStart", DoubleValue(16.0)); // Potencia de antena de dron (16 dBm).
    agrobotPhy.Set("TxPowerEnd", DoubleValue(16.0));

    YansWifiPhyHelper ugvPhy;
    ugvPhy.SetChannel(wifiChannel);
    ugvPhy.Set("TxPowerStart", DoubleValue(18.0)); // Los UGVs terrestres pueden llevar mejor batería (18 dBm).
    ugvPhy.Set("TxPowerEnd", DoubleValue(18.0));

    YansWifiPhyHelper sensorPhy;
    sensorPhy.SetChannel(wifiChannel);
    sensorPhy.Set("TxPowerStart", DoubleValue(12.0)); // Los sensores miniatura son débiles para ahorrar batería (12 dBm).
    sensorPhy.Set("TxPowerEnd", DoubleValue(12.0));

    YansWifiPhyHelper housePhy;
    housePhy.SetChannel(wifiChannel);
    housePhy.Set("TxPowerStart", DoubleValue(20.0)); // Antena directiva super fuerte de la base estática en la casa (20 dBm).
    housePhy.Set("TxPowerEnd", DoubleValue(20.0));

    // wifi.Install materializa los WifiNetDevice (Las tarjetas de red virtuales hardware/software) en cada grupo de nodos.
    NetDeviceContainer agrobotDevices = wifi.Install(agrobotPhy, mac, agrobotNodes);
    NetDeviceContainer ugvDevices = wifi.Install(ugvPhy, mac, ugvNodes);
    NetDeviceContainer sensorDevices = wifi.Install(sensorPhy, mac, sensors);
    NetDeviceContainer houseDevices = wifi.Install(housePhy, mac, house);

    // -------------------------------------------------------------------------
    // PILA IP Y ROUTING (Network Layer OSI Nivel 3)
    // -------------------------------------------------------------------------
    // AodvHelper se eligió porque la documentación de ns-3 lo posiciona como un
    // protocolo MANET reactivo (Ad-hoc On-demand Distance Vector) apropiado 
    // cuando la conectividad cambia velozmente por movilidad (como drones y tractores).
    // En este modelo AODV resuelve el encaminamiento IP cuando existe enlace,
    // pero no decide la oportunidad de forwarding: eso lo hace la app SCF por rango.
    AodvHelper aodv;
    InternetStackHelper stack; // Dota a los nodos vacios de una Pila IP con soporte TCP/UDP.
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    // Ipv4AddressHelper asigna direcciones secuenciales (10.1.0.1, .2, etc) sobre una sola subred,
    // suficiente porque el experimento comparte una única MANET ad hoc (Broadcast domain sin divisiones WAN ni VLans).
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");

    // Agrupa todas las tarjetas virtuales creadas antes en un paquete ordenado.
    NetDeviceContainer allDevices;
    allDevices.Add(agrobotDevices);
    allDevices.Add(ugvDevices);
    allDevices.Add(sensorDevices);
    allDevices.Add(houseDevices);
    // Inyecta las IPs del AddressHelper a la lista de tarjetas y lo retorna como un Ipv4InterfaceContainer (Base de datos DHCP virtual).
    Ipv4InterfaceContainer ifaces = ipv4.Assign(allDevices);

    // Guardamos offsets (saltos de memoria) dentro de Ipv4InterfaceContainer para recuperar direcciones
    // por rol sin depender de búsquedas posteriores que consumirían O(n) ciclos.
    const uint32_t ugvLeaderStartIndex = totalAgrobotNodes;
    const uint32_t sensorStartIndex = totalAgrobotNodes + totalUgvNodes;
    const uint32_t houseIndex = sensorStartIndex + sensorsPerField;

    // Separamos etapas lógicas por puerto UDP para que FlowMonitor pueda medirlas
    // por separado en Capa de Transporte a pesar de compartir la misma red IP en Capa de Red.
    const uint16_t sensorToL1Port = 9000;
    const uint16_t level1ToLevel2Port = 9001;
    const uint16_t level2ToHousePort = 9002;

    // Estas tablas enlazan roles experimentales abstractos con direcciones IP concretas asignadas por el simulador.
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

    // Arreglos de relaciones jerarquicas para armar la red estructurada lógicamente.
    std::vector<uint32_t> sensorAssignedAgrobotCluster(sensorsPerField);
    std::vector<uint32_t> sensorsPerAgrobotCluster(nLevel1Clusters, 0);
    std::vector<uint32_t> agrobotAssignedUgvCluster(nLevel1Clusters);
    std::vector<uint32_t> agrobotLeadersPerUgvCluster(nLevel2Clusters, 0);

    // Mapas para resolver a la inversa: Dado una IP -> Cual index es?.
    std::map<uint32_t, uint32_t> sensorIndexByIp;
    std::map<uint32_t, uint32_t> agrobotLeaderIndexByIp;
    std::map<uint32_t, uint32_t> ugvLeaderIndexByIp;

    // Emparejamiento entre niveles: cada líder agrobot tiene un cluster UGV
    // destino atado por configuración para el segundo relevo DTN.
    const std::vector<uint32_t> balancedAssignments =
        AssignAgrobotLeadersToUgvClusters(agrobotLeaders, ugvLeaders); // Invoca heurística de balanceo explicada antes.
    
    // Aplicación del balanceo
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

    // Cada sensor enterrado se asigna al líder agrobot (cuadrilla) más cercano al inicio de la corrida.
    // Esa decisión fija el camino lógico (Sensor -> Nivel 1) para toda la réplica, como estipula el experimento agrario.
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        const Vector sensorPosition = sensors.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const uint32_t agrobotClusterIndex = FindNearestNode(sensorPosition, agrobotLeaders);
        sensorAssignedAgrobotCluster[i] = agrobotClusterIndex;
        sensorsPerAgrobotCluster[agrobotClusterIndex]++;
        sensorIndexByIp.emplace(ifaces.GetAddress(sensorStartIndex + i).Get(), i);
    }

    // FlowMonitor observa flujos IP/UDP y permite reconstruir PDR por etapa lógica.
    // No ve estados internos del buffer, deduplicación ni AoI, así que esas
    // métricas siguen siendo responsabilidad de las aplicaciones SCF y de la casa.
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    // Arrays para guardar punteros a las APPs para consulta viva luego durante simulación.
    std::vector<Ptr<StoreCarryForwardApp>> sensorStoreApps;
    std::vector<Ptr<StoreCarryForwardApp>> agrobotRelayApps;
    std::vector<Ptr<StoreCarryForwardApp>> ugvRelayApps;
    Ptr<HouseCollectorApp> houseCollectorApp;
    
    // -------------------------------------------------------------------------
    // STORE-CARRY-FORWARD INSTANTIATION (Software Apps Deployment Nivel OSI 7)
    // -------------------------------------------------------------------------
    // Sensores, líderes agrobot y líderes UGV reciben apps distintas de configuración pero la
    // misma clase base (polimorfismo) StoreCarryForwardApp para mantener una semántica C++ única:
    // recibir, almacenar, transportar y reenviar por contactos.
    
    // Instalar en Sensores
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("sensor");
        app->SetForwardHop(0); // Nivel Base.
        // Hacia donde apunta el dardo lógico en el siguiente contacto aéreo
        app->SetNextHop(agrobotLeaders.Get(sensorAssignedAgrobotCluster[i]),
                        agrobotLeaderIps[sensorAssignedAgrobotCluster[i]],
                        sensorToL1Port);
        // Reglas de juego DTN.
        app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                            Seconds(sampleTtl),
                                            Seconds(contactCheckInterval),
                                            forwardBurstPackets,
                                            sensorContactRange);
        // Como son sensores, tienen rol de SOURCE creador de tráfico.
        app->SetSourceParameters(i,
                                 Seconds(sensorInterval),
                                 packetSize,
                                 Seconds(1.5 + 0.05 * static_cast<double>(i % 20))); // Jitter inicial para evitar colisión mágica de MAC en t=0.0
        sensors.Get(i)->AddApplication(app);
        // Start/StopTime siguen la convención de ns-3 para que la aplicación
        // entre al ciclo de vida del simulador en lugar de arrancar "a mano".
        app->SetStartTime(Seconds(0.5)); // Empieza tras estabilización PHY.
        app->SetStopTime(Seconds(simTime - 0.5)); // Acaba un pelito antes de la destrucción total para drenar.
        sensorStoreApps.push_back(app);
    }

    // Instalar en Agrobots
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("agrobot-relay"); // Rol repetidor Nivel 1.
        app->SetListenPort(sensorToL1Port); // Es sumidero de los sensores locales.
        app->SetForwardHop(1); // Es transmisor hacia la siguiente jerarquía.
        // Hacia el Tractor de suelo correspondiente.
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

    // Instalar en Tractores UGVs Terrestres
    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
        app->SetRoleName("ugv-relay");
        app->SetListenPort(level1ToLevel2Port); // Escucha a los drones Agrobots.
        app->SetForwardHop(2);
        // Hacia la Antena Fija de Comando (La Casa).
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

    // Instalar HouseApp
    houseCollectorApp = CreateObject<HouseCollectorApp>();
    // La casa solo escucha en el último puerto y calcula métricas extremo a extremo. ¡NO REENVIA NADA! Es el fin del viaje.
    houseCollectorApp->Configure(level2ToHousePort, sensorsPerField, Seconds(aoiSamplePeriod));
    house.Get(0)->AddApplication(houseCollectorApp);
    houseCollectorApp->SetStartTime(Seconds(0.5));
    houseCollectorApp->SetStopTime(Seconds(simTime));

    // -------------------------------------------------------------------------
    // EXPORTACION TEMPORAL (Logs Vivos Durante Ejecución)
    // -------------------------------------------------------------------------
    // La serie temporal queda acotada a las métricas que alimentan los
    // dashboards: cobertura intermedia/final y frescura observada en la casa.
    // Procedencia:
    // - agrobot_coverage_pct: cálculo manual sobre sensores que ya alcanzaron el primer relevo.
    // - house_coverage_pct: cálculo manual sobre sensores con al menos una entrega en casa.
    // - house_aoi_avg_s: cálculo manual en HouseCollectorApp sobre la actualización más fresca visible.
    // Preparar el esquema del archivo .CSV
    std::vector<std::string> timeseriesHeaderFields = {"run_label",
                                                       "time_s",
                                                       "agrobot_coverage_pct",
                                                       "house_coverage_pct",
                                                       "house_aoi_avg_s"};

    EnsureCsvHeader(timeseriesCsvFile, JoinCsvFields(timeseriesHeaderFields));

    // snapshotMetrics captura el estado observado de RAM en un instante fijo en medio de la simulación.
    // Se programa periódicamente de forma recursiva (como un hilo de daemon de medición) 
    // y escribe una fila física al disco rígido por muestra temporal.
    // Se utiliza sintaxis Lambda de C++ moderno `[&]()` para capturar el contexto de ejecución global por referencia.
    std::function<void()> snapshotMetrics;
    snapshotMetrics = [&]() {
        const double nowSeconds = Simulator::Now().GetSeconds();

        std::vector<std::string> row = {metricsRunLabel, FormatDouble(nowSeconds, 3)};

        // Calculo de porcentajes vivos leyendo RAM de los objetos Apps atados a los nodos.
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

        // Guarda línea nueva en el archivo.
        AppendCsvLine(timeseriesCsvFile, JoinCsvFields(row));

        // Si aún nos queda simulacion de vida, la Lambda se agenda de nuevo a sí misma en el simulador.
        if (nowSeconds + metricsSampleInterval <= simTime)
        {
            Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);
        }
    };

    // Dispara el primer golpe de la cadena Lambda.
    Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);

    // -------------------------------------------------------------------------
    // MOTOR DE SIMULACIÓN (EL MOTOR DISCRETO DE EVENTOS EMPIEZA AQUI)
    // -------------------------------------------------------------------------
    // La corrida termina exactamente en simTime; las apps ya tienen StopTime algo
    // anterior para dejar un margen de drenaje antes del cierre global y no romper archivos ni logs.
    Simulator::Stop(Seconds(simTime)); // Ordena la muerte de la simulación.
    Simulator::Run(); // Transfiere control de CPU a ns-3 hasta terminar la cola de eventos.

    // -------------------------------------------------------------------------
    // POST-PROCESAMIENTO TRAS FINALIZAR EL TIEMPO `simTime`
    // -------------------------------------------------------------------------

    // CheckForLostPackets fuerza a FlowMonitor a contabilizar pérdidas pendientes en colas PHY MAC
    // que aún no expiraban antes de que leamos las estadísticas finales para ser precisos.
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    const std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::vector<bool> agrobotLeadersDelivered(nLevel1Clusters, false);
    std::vector<bool> ugvLeadersDelivered(nLevel2Clusters, false);

    // Reconstruimos las etapas lógicas (Etapa 2 y Etapa 3 IP) que luego se exponen en los dashboards
    // finales para análisis de Packet Delivery Ratio PDR. Estas métricas provienen de FlowMonitor
    // filtrando por puerto UDP, así que representan desempeño IP por etapa y no estado interno SCF.
    // La etapa 1 sigue disponible vía cobertura temporal agrobot.
    const StageMetrics stage2Metrics =
        CollectStageMetrics(stats,
                            classifier,
                            level1ToLevel2Port, // Escucha la ruta Aerea Terrestre
                            agrobotLeaderIndexByIp,
                            agrobotLeadersDelivered);
    const StageMetrics stage3Metrics =
        CollectStageMetrics(
            stats, classifier, level2ToHousePort, ugvLeaderIndexByIp, ugvLeadersDelivered); // Terrestre a Fija

    // -------------------------------------------------------------------------
    // REPORTE EN CONSOLA (Interfaz STDOUT de la Simulación al Usuario)
    // -------------------------------------------------------------------------
    // El log queda alineado con los dashboards para no exportar telemetría en terminal que
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

    // Extrae estadísticas globales llamando la lógica de capa y sumatoria de Buffer.
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

    // El XML conserva trazabilidad detallada por flujo de red IP (Paquete por Paquete, Jitter, Drops)
    // para análisis masivo posterior offline, vital en caso de comportamiento anómalo.
    monitor->SerializeToXmlFile(flowmonFile, true, true);

    // -------------------------------------------------------------------------
    // CSV RESUMEN FINAL
    // -------------------------------------------------------------------------
    // Una fila por corrida completa. Se exporta solo el subconjunto escalar que consumen los
    // dashboards de graficación comparativos de rango, velocidad y TTL+buffer para análisis científico.
    // Procedencia:
    // - stage2_pdr_pct y stage3_pdr_pct: derivados de FlowMonitor.
    // - house_coverage_pct, house_e2e_delay_avg_s y house_aoi_avg_s: calculados en HouseCollectorApp.
    // - *_ttl_drops y *_avg_buffer_occ: agregados manualmente desde StoreCarryForwardApp por capa.
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

    // Crea cabeceras si el disco está vacío o es un archivo nuevo.
    EnsureCsvHeader(summaryCsvFile, JoinCsvFields(summaryHeaderFields));

    // La fila se arma en el mismo orden exacto que el array del header para que el archivo pueda
    // consumirse (Por Pandas, matplotlib) sin inferencias externas.
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

    // Imprime la cadena plana separada por comas en el archivo final de base de datos plana.
    AppendCsvLine(summaryCsvFile, JoinCsvFields(summaryRow));

    // Destroy libera objetos singleton ns-3 (Time, EventQueues) de la memoria Heap  y
    // deja el ambiente C++ listo por si se vuelve a iterar la corrida de forma incrustada.
    Simulator::Destroy();
    return 0; // Código 0, Finalización sin fallos de sistema (Unix Standard).
}
