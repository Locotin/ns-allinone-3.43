#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/ssid.h"
#include "ns3/wifi-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/netanim-module.h"
#include "ns3/interference-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h" // REQUERIDO PARA ALARMA ALEATORIA

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream> // Para exportar métricas
#include <iomanip> // Para formato de salida
#include <map>     // REQUERIDO PARA ESTADO DEL DRON
#include <limits>  // REQUERIDO PARA ESTADO DEL DRON
#include <sstream>
#include <vector>

namespace ns3
{

class MultiZonePropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();

    MultiZonePropagationLossModel() = default;

    void AddCircularZone(double centerX,
                         double centerY,
                         double radius,
                         double referenceLossDb,
                         double exponent)
    {
        m_zones.push_back({Vector(centerX, centerY, 0.0), radius, referenceLossDb, exponent});
    }

    void SetDefaultZone(double referenceLossDb, double exponent)
    {
        m_defaultZone = {Vector(0.0, 0.0, 0.0), 0.0, referenceLossDb, exponent};
    }

    void SetRainEffect(bool enabled)
    {
        m_rainEffect = enabled;
    }

    double GetLoss(const Vector& txPos, const Vector& rxPos) const
    {
        const Zone* zone = SelectZone(txPos, rxPos);
        const double distance = std::max(CalculateDistance(txPos, rxPos), m_minDistanceMeters);
        double lossDb = zone->referenceLossDb + 10.0 * zone->exponent * std::log10(distance);
        if (m_rainEffect)
        {
            lossDb += m_rainPenaltyDb;
        }
        return lossDb;
    }

  private:
    struct Zone
    {
        Vector center;
        double radius;
        double referenceLossDb;
        double exponent;
    };

    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
    {
        return txPowerDbm - GetLoss(a->GetPosition(), b->GetPosition());
    }

    int64_t DoAssignStreams(int64_t stream) override
    {
        return 0;
    }

    const Zone* SelectZone(const Vector& txPos, const Vector& rxPos) const
    {
        Vector midpoint((txPos.x + rxPos.x) / 2.0,
                        (txPos.y + rxPos.y) / 2.0,
                        (txPos.z + rxPos.z) / 2.0);

        for (const auto& zone : m_zones)
        {
            if (IsInsideZone(midpoint, zone) || IsInsideZone(txPos, zone) || IsInsideZone(rxPos, zone))
            {
                return &zone;
            }
        }

        return &m_defaultZone;
    }

    static bool IsInsideZone(const Vector& point, const Zone& zone)
    {
        return CalculateDistance(point, zone.center) <= zone.radius;
    }

    static double CalculateDistance(const Vector& a, const Vector& b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<Zone> m_zones;
    Zone m_defaultZone{Vector(0.0, 0.0, 0.0), 0.0, 46.0, 2.5};
    bool m_rainEffect{false};
    double m_rainPenaltyDb{8.0};
    double m_minDistanceMeters{1.0};
};

NS_OBJECT_ENSURE_REGISTERED(MultiZonePropagationLossModel);

TypeId
MultiZonePropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MultiZonePropagationLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("Propagation")
            .AddConstructor<MultiZonePropagationLossModel>();
    return tid;
}

} // namespace ns3

using namespace ns3;

namespace
{

struct BufferedPacket
{
    Ipv4Address dest;
    std::string msg;
    Ptr<Packet> packet;
};

using PacketQueue = std::deque<BufferedPacket>;

std::map<uint32_t, PacketQueue> g_bufferedPackets;
std::map<uint32_t, PacketQueue> g_bufferedPacketsSent;

PacketQueue&
GetQueue(std::map<uint32_t, PacketQueue>& queues, const Ptr<Node>& node)
{
    return queues[node->GetId()];
}

void
BufferPacket(const Ptr<Node>& node, Ipv4Address dest, const std::string& msg, const Ptr<Packet>& packet)
{
    GetQueue(g_bufferedPackets, node).push_back({dest, msg, packet->Copy()});
}

bool
HasBufferedPackets(const Ptr<Node>& node)
{
    const auto it = g_bufferedPackets.find(node->GetId());
    return it != g_bufferedPackets.end() && !it->second.empty();
}

BufferedPacket
PeekBufferedPacket(const Ptr<Node>& node)
{
    return g_bufferedPackets.at(node->GetId()).front();
}

void
PopBufferedPacket(const Ptr<Node>& node)
{
    auto& queue = g_bufferedPackets.at(node->GetId());
    queue.pop_front();
}

void
BufferPacketSent(const Ptr<Node>& node,
                 Ipv4Address dest,
                 const std::string& msg,
                 const Ptr<Packet>& packet)
{
    GetQueue(g_bufferedPacketsSent, node).push_back({dest, msg, packet->Copy()});
}

bool
HasBufferedPacketsSent(const Ptr<Node>& node)
{
    const auto it = g_bufferedPacketsSent.find(node->GetId());
    return it != g_bufferedPacketsSent.end() && !it->second.empty();
}

BufferedPacket
PeekBufferedPacketSent(const Ptr<Node>& node)
{
    return g_bufferedPacketsSent.at(node->GetId()).front();
}

void
PopBufferedPacketSent(const Ptr<Node>& node)
{
    auto& queue = g_bufferedPacketsSent.at(node->GetId());
    queue.pop_front();
}

} // namespace

// ---------------------------
//  Globales
// ---------------------------
static Ptr<MultiZonePropagationLossModel> g_multiZoneModel = 0;
static AnimationInterface* g_anim = nullptr; // Puntero global de NetAnim
static Ptr<UniformRandomVariable> g_rand;    // Generador aleatorio para alarmas
// Mapea cada Cluster Head a sus nodos (por ID de nodo)
static std::map<uint32_t, NodeContainer> g_clusterNodesMap;
// Mapea índice de cluster (0,1,2) a sus nodos sensores - PARA MÉTRICAS
static std::map<uint32_t, NodeContainer> g_clusterByIndexMap;

Ipv4Address g_superAddr; 


// Mapa para rastrear el "próximo evento de patrulla" de cada dron.
static std::map< Ptr<ConstantVelocityMobilityModel>, EventId > g_droneNextPatrolEvent;

//  CORRECCIÓN DE COMPILACIÓN: Era std::map, no std.map
static std::map< Ptr<ConstantVelocityMobilityModel>, uint32_t > g_dronePatrolIndex;

// ---------------------------
//  ESTRUCTURAS PARA MÉTRICAS MEJORADAS
// ---------------------------
struct ClusterMetrics {
    uint32_t packetsSent = 0;
    uint32_t packetsDelivered = 0;
    uint32_t packetsLost = 0;
    std::vector<double> latencies;
    double totalLatency = 0.0;
    uint32_t packetsUnderRain = 0;
    uint32_t deliveredUnderRain = 0;
    
    double GetPDR() const {
        return packetsSent > 0 ? (100.0 * packetsDelivered / packetsSent) : 0.0;
    }
    
    double GetAvgLatency() const {
        return latencies.empty() ? 0.0 : (totalLatency / latencies.size());
    }
    
    double GetPacketLossRate() const {
        return packetsSent > 0 ? (100.0 * packetsLost / packetsSent) : 0.0;
    }
};

struct InterClusterMetrics {
    std::vector<double> latenciesAtoB;  // Cluster A -> B
    std::vector<double> latenciesAtoC;  // Cluster A -> C
    std::vector<double> latenciesBtoA;  // Cluster B -> A
    std::vector<double> latenciesBtoC;  // Cluster B -> C
    std::vector<double> latenciesCtoA;  // Cluster C -> A
    std::vector<double> latenciesCtoB;  // Cluster C -> B
    
    double GetAvgLatency(const std::vector<double>& lats) const {
        if (lats.empty()) return 0.0;
        double sum = 0.0;
        for (double lat : lats) sum += lat;
        return sum / lats.size();
    }
};

struct SimulationMetrics {
    // Paquetes globales
    uint32_t totalPacketsSent = 0;
    uint32_t packetsDeliveredToSuper = 0;
    uint32_t packetsLost = 0;
    uint32_t packetsBuffered = 0;
    
    // Latencias globales
    std::vector<double> latencies;
    double totalLatency = 0.0;
    double minLatency = std::numeric_limits<double>::max();
    double maxLatency = 0.0;
    
    // Métricas por cluster (0=A, 1=B, 2=C)
    std::map<uint32_t, ClusterMetrics> clusterMetrics;
    
    // Métricas inter-cluster
    InterClusterMetrics interCluster;
    
    // Conectividad
    uint32_t connectivityChecks = 0;
    uint32_t successfulConnections = 0;
    
    // Lluvia (CORREGIDO: ahora trackea correctamente)
    double timeUnderRain = 0.0;
    uint32_t packetsUnderRain = 0;      // Enviados bajo lluvia
    uint32_t deliveredUnderRain = 0;     // Entregados que fueron enviados bajo lluvia
    uint32_t packetsSentNoRain = 0;      // Enviados sin lluvia
    uint32_t deliveredNoRain = 0;        // Entregados que fueron enviados sin lluvia
    
    // Drones
    uint32_t droneInterceptions = 0;
    uint32_t alarmsTriggers = 0;
    
    // Timestamps y trazabilidad
    std::map<uint32_t, double> packetSendTime;        // packet UID -> send time
    std::map<uint32_t, bool> packetSentInRain;        // packet UID -> was sent during rain
    std::map<uint32_t, uint32_t> packetSourceCluster; // packet UID -> source cluster (0=A,1=B,2=C)
    std::map<uint32_t, uint32_t> packetDestCluster;   // packet UID -> dest cluster for inter-cluster
    
    // Cálculos
    double GetPDR() const {
        return totalPacketsSent > 0 ? (100.0 * packetsDeliveredToSuper / totalPacketsSent) : 0.0;
    }
    
    double GetPacketLossRate() const {
        return totalPacketsSent > 0 ? (100.0 * packetsLost / totalPacketsSent) : 0.0;
    }
    
    double GetAverageLatency() const {
        return latencies.empty() ? 0.0 : (totalLatency / latencies.size());
    }
    
    double GetLatencyStdDev() const {
        if (latencies.size() < 2) return 0.0;
        double avg = GetAverageLatency();
        double variance = 0.0;
        for (double lat : latencies) {
            variance += (lat - avg) * (lat - avg);
        }
        return std::sqrt(variance / latencies.size());
    }
    
    double GetConnectivityRate() const {
        return connectivityChecks > 0 ? (100.0 * successfulConnections / connectivityChecks) : 0.0;
    }
    
    // CORREGIDO: PDR bajo lluvia solo de paquetes enviados bajo lluvia
    double GetRainPDR() const {
        return packetsUnderRain > 0 ? (100.0 * deliveredUnderRain / packetsUnderRain) : 0.0;
    }
    
    // PDR sin lluvia
    double GetNoRainPDR() const {
        return packetsSentNoRain > 0 ? (100.0 * deliveredNoRain / packetsSentNoRain) : 0.0;
    }
};

static SimulationMetrics g_metrics;
static bool g_isRaining = false;
static uint32_t g_nextPacketUid = 1;


NS_LOG_COMPONENT_DEFINE("ManetRecolector");

// ---------------------------
//  Declaración de funciones
// ---------------------------
Ptr<YansWifiChannel> CreateMultiZoneChannel();
void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy);
void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, WifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolectors,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower); // [NUEVO]

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C);
void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid);
// Nota: ahora recibe A,B,C para distribuir inicios
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolectors, MobilityHelper &mobility, Vector A, Vector B, Vector C); // [MODIFICADA]

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolectors);

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);

void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port,
                    Ipv4Address finalDest, const std::string &msg);
void SendUploadData(Ptr<Node> from, Ipv4Address toAddr);
void SendForwardMessage(Ptr<Node> from, Ipv4Address toAddr);
void SendStatusEvent(Ptr<Node> from, Ipv4Address toAddr, std::string msg);

void InstallUdpServers(NodeContainer &sensors, NodeContainer &recolectors, NodeContainer &superCluster, NodeContainer &clusterHeads, double simTime);
// Función para instalar un socket "listener" en cada ClusterHead
void InstallClusterHeadReceivers(NodeContainer &clusterHeads, uint16_t listenPort = 5000);
void SetupIntraClusterClients(NodeContainer &sensors,NodeContainer &clusterHeads,Ipv4InterfaceContainer &interfaces, double simTime);
// ScheduleRecolectorMovement ahora recibe simEnd para repetir hasta el final
void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed, double simEnd, Vector centroid);

void RunSimulation(double simTime, bool enableRain = true);

void PeriodicProximityCheck(NodeContainer &clusterHeads,NodeContainer &recolectors,double checkInterval);
static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime, double patrolSpeed);
static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime, double leadTime = 5.0, double patrolSpeed = 5.0);

double Dist2D(const Vector &a, const Vector &b);
void SchedulePeriodicProximityCheck(NodeContainer &clusterHeads,NodeContainer &recolector,double checkInterval = 1.0);

// Funciones para análisis de métricas
void PrintMetricsReport(const std::string& scenarioName, double simTime, uint32_t nDrones);
void ExportMetricsToCSV(const std::string& filename, const std::string& scenarioName, double simTime, uint32_t nDrones);
void AnalyzeCoverageArea(NodeContainer& nodes, Vector A, Vector B, Vector C);

// -----------------------------------------------------------------
//  LÓGICA DE MOVIMIENTO Y ALARMA (SIMULADA) 
// -----------------------------------------------------------------

// --- Declaración de funciones de la nueva lógica ---

double SetVelocityTowards(Ptr<ConstantVelocityMobilityModel> mv, Vector to, double speed);
void PatrolWaypointReached(Ptr<ConstantVelocityMobilityModel> mv, 
                           std::vector<Vector> route, double speed);
void StartPatrolling(Ptr<ConstantVelocityMobilityModel> mv, 
                     std::vector<Vector> route, double speed);
void ResumePatrolAfterAlarm(Ptr<ConstantVelocityMobilityModel> mv, Ptr<Node> droneNode, 
                            Ptr<Node> alarmNode, std::vector<Vector> route, double speed);
void TriggerRandomAlarm(NodeContainer sensors, NodeContainer recolector, 
                        std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs,
                        std::vector<Vector> route, double patrolSpeed, double alarmSpeed, double simTime);
std::pair<Ptr<ConstantVelocityMobilityModel>, Ptr<Node>> 
FindClosestDrone(Vector targetPos, NodeContainer& recolector, std::vector<Ptr<ConstantVelocityMobilityModel>>& recolectorMVs);

// --- Implementación de funciones de la nueva lógica ---

double SetVelocityTowards(Ptr<ConstantVelocityMobilityModel> mv, Vector to, double speed)
{
    Vector from = mv->GetPosition();
    Vector dir = to - from;
    double dist = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (dist < 1e-6)
    {
        mv->SetVelocity(Vector(0,0,0));
        return 0.0;
    }

    Vector vel = Vector(dir.x / dist * speed, dir.y / dist * speed, dir.z / dist * speed);
    mv->SetVelocity(vel);
    
    return dist / speed; // Devuelve el tiempo del trayecto
}

void PatrolWaypointReached(Ptr<ConstantVelocityMobilityModel> mv, 
                           std::vector<Vector> route, double speed)
{
    // 1. Averiguar dónde estamos y a dónde vamos
    uint32_t currentIndex = g_dronePatrolIndex[mv];
    uint32_t nextIndex = (currentIndex + 1) % route.size();
    
    // 2. Establecer la velocidad para el *siguiente* tramo
    double timeToNext = SetVelocityTowards(mv, route[nextIndex], speed);
    
    // 3. Almacenar el *nuevo* destino
    g_dronePatrolIndex[mv] = nextIndex;

    // 4. Programar esta *misma* función para que se llame al llegar al *siguiente* punto
    EventId nextEvent = Simulator::Schedule(Seconds(timeToNext), 
                                           &PatrolWaypointReached, 
                                           mv, route, speed);
    
    // 5. Almacenar el ID del evento para poder cancelarlo si hay una alarma
    g_droneNextPatrolEvent[mv] = nextEvent;
}

void StartPatrolling(Ptr<ConstantVelocityMobilityModel> mv, 
                     std::vector<Vector> route, double speed)
{
    // 1. Encontrar el punto de patrulla (A, B, o C) más cercano al punto de inicio del dron
    Vector startPos = mv->GetPosition();
    double minDist = std::numeric_limits<double>::max();
    uint32_t closestIndex = 0;

    for (uint32_t i = 0; i < route.size(); ++i)
    {
        Vector dir = route[i] - startPos;
        double dist = dir.x * dir.x + dir.y * dir.y; // sqr dist
        if (dist < minDist)
        {
            minDist = dist;
            closestIndex = i;
        }
    }

    // 2. Establecer la velocidad hacia ese primer punto
    double timeToFirst = SetVelocityTowards(mv, route[closestIndex], speed);

    // 3. Almacenar ese punto como nuestro destino actual
    g_dronePatrolIndex[mv] = closestIndex;

    // 4. Programar el bucle de patrulla para que comience al *llegar* a ese primer punto
    EventId firstEvent = Simulator::Schedule(Seconds(timeToFirst), 
                                             &PatrolWaypointReached, 
                                             mv, route, speed);

    g_droneNextPatrolEvent[mv] = firstEvent;
}

void ResumePatrolAfterAlarm(Ptr<ConstantVelocityMobilityModel> mv, Ptr<Node> droneNode, 
                            Ptr<Node> alarmNode, std::vector<Vector> route, double speed)
{
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Dron " << droneNode->GetId() 
                << " llegó a la alarma en Nodo " << alarmNode->GetId() << ". Reanudando patrulla.");

    // 1. Resetear colores en NetAnim
    if (g_anim)
    {
        // Resetea el dron a su color original (azul)
        g_anim->UpdateNodeColor(droneNode, 0, 0, 255);
        // Resetea el sensor a su color original (azul)
        g_anim->UpdateNodeColor(alarmNode, 0, 0, 255); 
    }

    // 2. Recuperar el destino de patrulla que tenía *antes* de la alarma
    uint32_t targetIndex = g_dronePatrolIndex[mv];
    Vector targetPatrolPos = route[targetIndex];

    // 3. Establecer velocidad hacia ese destino de patrulla
    double timeToPatrol = SetVelocityTowards(mv, targetPatrolPos, speed);
    
    // 4. Programar el bucle de patrulla para que se active al llegar
    EventId nextEvent = Simulator::Schedule(Seconds(timeToPatrol), 
                                           &PatrolWaypointReached, 
                                           mv, route, speed);
    
    g_droneNextPatrolEvent[mv] = nextEvent;
}

std::pair<Ptr<ConstantVelocityMobilityModel>, Ptr<Node>> 
FindClosestDrone(Vector targetPos, NodeContainer& recolector, std::vector<Ptr<ConstantVelocityMobilityModel>>& recolectorMVs)
{
    double minDist = std::numeric_limits<double>::max();
    Ptr<ConstantVelocityMobilityModel> closestMv = nullptr;
    Ptr<Node> closestNode = nullptr;

    for (uint32_t i = 0; i < recolectorMVs.size(); ++i)
    {
        Vector pos = recolectorMVs[i]->GetPosition();
        Vector dir = targetPos - pos;
        double dist = dir.x * dir.x + dir.y * dir.y; // sqr dist
        
        if (dist < minDist)
        {
            minDist = dist;
            closestMv = recolectorMVs[i];
            closestNode = recolector.Get(i);
        }
    }
    return {closestMv, closestNode};
}


void TriggerRandomAlarm(NodeContainer sensors, NodeContainer recolector, 
                        std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs,
                        std::vector<Vector> route, double patrolSpeed, double alarmSpeed, double simTime)
{
    //  Registrar alarma
    g_metrics.alarmsTriggers++;
    
    // --- 1. Generar la alarma ---
    uint32_t sensorIndex = g_rand->GetInteger(0, sensors.GetN() - 1);
    Ptr<Node> alarmNode = sensors.Get(sensorIndex);
    Vector alarmPos = alarmNode->GetObject<MobilityModel>()->GetPosition();

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: 🚨 ¡ALARMA! Generada en Nodo Sensor " 
                << alarmNode->GetId() << " en (" << alarmPos.x << "," << alarmPos.y << ")");

    // --- 2. Encontrar el dron que responde ---
    auto [closestMv, closestNode] = FindClosestDrone(alarmPos, recolector, recolectorMVs);

    if (!closestMv) 
    {
        NS_LOG_WARN("Alarma generada, pero no hay drones para responder.");
        return;
    }
    
    NS_LOG_INFO("... Dron " << closestNode->GetId() << " es el más cercano y responderá.");

    //  Registrar intercepción de dron
    g_metrics.droneInterceptions++;

    // --- 3. Interrumpir la patrulla del dron ---
    EventId patrolEvent = g_droneNextPatrolEvent[closestMv];
    
    if (patrolEvent.IsPending())
    {
        Simulator::Cancel(patrolEvent);
        NS_LOG_INFO("... Patrulla del Dron " << closestNode->GetId() << " cancelada.");
    }
    
    // --- 4. Enviar dron a la alarma ---
    double timeToAlarm = SetVelocityTowards(closestMv, alarmPos, alarmSpeed);
    
    // --- 5. Actualizar NetAnim ---
    if (g_anim)
    {
        g_anim->UpdateNodeColor(closestNode, 0, 255, 0); // Dron que responde = VERDE
        g_anim->UpdateNodeColor(alarmNode, 255, 255, 0); // Nodo de alarma = AMARILLO
    }
    
    // --- 6. Programar la "reanudación de patrulla" ---
    Simulator::Schedule(Seconds(timeToAlarm), &ResumePatrolAfterAlarm, 
                        closestMv, closestNode, alarmNode, 
                        route, patrolSpeed);

    // --- 7. Programar la *próxima* alarma aleatoria ---
    double nextAlarmIn = g_rand->GetValue(15.0, 25.0); 
    if (Simulator::Now().GetSeconds() + nextAlarmIn < (simTime - 10.0)) // No programar si es muy al final
    {
        Simulator::Schedule(Seconds(nextAlarmIn), &TriggerRandomAlarm,
                            sensors, recolector, recolectorMVs, 
                            route, patrolSpeed, alarmSpeed, simTime);
    }
}


// ---------------------------
// 🔸 Implementación
// ---------------------------

Ptr<YansWifiChannel> CreateMultiZoneChannel()
{
    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();

    // Modelo multi-zona
    Ptr<MultiZonePropagationLossModel> multiZone = CreateObject<MultiZonePropagationLossModel>();

    // ============================
    //  Definición de zonas (círculos)
    // ============================
    // Cluster A - zona con poca pérdida (campo abierto)
    multiZone->AddCircularZone(0.0, 0.0, 40.0, 51.0, 2.3);

    // Cluster B - zona intermedia
    multiZone->AddCircularZone(100.0, 0.0, 40.0, 59.0, 2.0);

    // Cluster C - zona densa con árboles
    multiZone->AddCircularZone(50.0, 86.6025403784, 40.0, 50.0, 3.5);

    // Zona por defecto (fuera de los clusters)
    multiZone->SetDefaultZone(46.0, 2.5);


    // Guardar el puntero global para poder activarle efectos dinámicos (lluvia)
    g_multiZoneModel = multiZone;

    // Configurar el canal WiFi
    channel->SetPropagationLossModel(multiZone);
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    return channel;
}

void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy)
{
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifiMac.SetType("ns3::AdhocWifiMac");

    // Crear canal usando el método auxiliar
    Ptr<YansWifiChannel> channel = CreateMultiZoneChannel();

    // Asignar el canal físico
    wifiPhy.SetChannel(channel);

    // Configuración base de potencia para nodos normales (AUMENTADA para mejor PDR)
    wifiPhy.Set("TxPowerStart", DoubleValue(23.0)); // Aumentado de 16 a 23 dBm
    wifiPhy.Set("TxPowerEnd", DoubleValue(23.0));   // ~200 metros de rango teórico
}

void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolectors,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower) // [NUEVO]
{
    // Nodos sensores (potencia normal)
    sensorDevices = wifi.Install(wifiPhy, wifiMac, sensors);

    // Si la opción está activada, los líderes usan más potencia
    if (useLeaderSignalPower)
    {
        NS_LOG_INFO("Cluster Heads con mayor potencia de transmisión activado.");
        YansWifiPhyHelper highPowerPhy = wifiPhy;
        highPowerPhy.Set("TxPowerStart", DoubleValue(27.0)); // Aumentado de 20 a 27 dBm
        highPowerPhy.Set("TxPowerEnd", DoubleValue(27.0));   // ~300 metros de rango
        headDevices = wifi.Install(highPowerPhy, wifiMac, clusterHeads);
    }
    else
    {
        headDevices = wifi.Install(wifiPhy, wifiMac, clusterHeads);
    }

    // Super Cluster y recolectores usan potencia base
    superDevices = wifi.Install(wifiPhy, wifiMac, superCluster);
    recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolectors);

    // Log: mostrar potencia TX actual de cada cluster head (útil para verificar)
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i) {
        Ptr<NetDevice> nd = headDevices.Get(i);
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(nd);
        if (dev) {
            Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(dev->GetPhy());
            if (phy) {
                NS_LOG_INFO("Head " << i << " potencia TX = " << phy->GetTxPowerStart());
            }
        }
    }
}

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C)
{
    auto setupCluster = [&](Vector center, uint32_t startIndex, uint32_t endIndex)
    {
        Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
        posAlloc->Add(Vector(center.x + 5.0, center.y + 0.0, 0));
        posAlloc->Add(Vector(center.x - 5.0, center.y + 3.0, 0));
        posAlloc->Add(Vector(center.x + 0.0, center.y - 4.0, 0));

        MobilityHelper mob;
        mob.SetPositionAllocator(posAlloc);
        mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue("Time"),
                             "Time", TimeValue(Seconds(2.0)),
                             "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                             "Bounds", RectangleValue(Rectangle(center.x - 10.0, center.x + 10.0,
                                                                 center.y - 10.0, center.y + 10.0)));
        NodeContainer group;
        for (uint32_t i = startIndex; i <= endIndex; ++i)
            group.Add(sensors.Get(i));
        mob.Install(group);
    };

    setupCluster(A, 0, 2);
    setupCluster(B, 3, 5);
    setupCluster(C, 6, 8);
}

void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid)
{
    Ptr<ListPositionAllocator> headPositions = CreateObject<ListPositionAllocator>();
    headPositions->Add(A);
    headPositions->Add(B);
    headPositions->Add(C);
    mobility.SetPositionAllocator(headPositions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(clusterHeads);

    Ptr<ListPositionAllocator> superPos = CreateObject<ListPositionAllocator>();
    superPos->Add(centroid);
    mobility.SetPositionAllocator(superPos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(superCluster);
}

// [MODIFICADA] Configura movilidad para N recolectores y devuelve sus modelos
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolectors, MobilityHelper &mobility, Vector A, Vector B, Vector C)
{
    std::vector<Ptr<ConstantVelocityMobilityModel>> movers;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolectors);

    // Bases de inicio: A, B, C (cíclico si hay más drones)
    std::vector<Vector> bases = {A, B, C};

    for (uint32_t i = 0; i < recolectors.GetN(); ++i) {
        Ptr<Node> n = recolectors.Get(i);
        Ptr<ConstantVelocityMobilityModel> mv = n->GetObject<ConstantVelocityMobilityModel>();
        // selección base según índice
        Vector base = bases[i % bases.size()];
        // pequeño offset para evitar coincidencia exacta de posiciones
        double offset = 3.0 * i;
        mv->SetPosition(Vector(base.x + offset, base.y + 0.5 * offset, base.z));
        movers.push_back(mv);
        NS_LOG_INFO("Recolector " << i << " pos inicial = (" << mv->GetPosition().x << "," << mv->GetPosition().y << ")");
    }
    return movers;
}

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolectors)
{
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(sensors);
    stack.Install(clusterHeads);
    stack.Install(superCluster);
    stack.Install(recolectors);
}

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices)
{
    address.SetBase("10.0.0.0", "255.255.255.0");
    NetDeviceContainer allDevices;
    allDevices.Add(sensorDevices);
    allDevices.Add(headDevices);
    allDevices.Add(superDevices);
    allDevices.Add(recolectorDevices);
    return address.Assign(allDevices);
}


void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port,
                    Ipv4Address finalDest, const std::string &msg)
{
    if (from == nullptr) return;

    // NOTA: Las métricas se registran SOLO en SendStatusEvent (envío original del sensor)
    // Esta función se usa para REENVÍOS (CH→Recolector→SuperCluster)
    // El mensaje YA contiene el UID original y cluster, solo lo propagamos
    
    // Construir payload directamente con el mensaje recibido (que ya tiene UID y cluster)
    std::ostringstream oss;
    oss << finalDest << "|" << msg;
    std::string payload = oss.str();

    Ptr<Packet> packet = Create<Packet>((uint8_t*)payload.c_str(), payload.size());
    if (packet == nullptr) return;

    // Crear socket UDP
    Ptr<Socket> socket = Socket::CreateSocket(from, UdpSocketFactory::GetTypeId());
    if (socket == nullptr) return;

    // Conectar y enviar
    if (socket->Connect(InetSocketAddress(toAddr, port)) != 0) return;

    socket->Send(packet);

    // Cerrar el socket después de enviar
    socket->Close();
}

void SendStatusEvent(Ptr<Node> from, Ipv4Address chAddr, Ipv4Address finalDest, std::string msg)
{
    uint16_t port = 4000; // Puerto del CH

    // REGISTRAR MÉTRICAS - Obtener IP del sensor desde el nodo 'from'
    Ptr<Ipv4> ipv4 = from->GetObject<Ipv4>();
    Ipv4Address sensorAddr = ipv4->GetAddress(1, 0).GetLocal(); // Interfaz 1 = WiFi
    
    uint32_t packetUid = g_nextPacketUid++;
    g_metrics.totalPacketsSent++;
    g_metrics.packetSendTime[packetUid] = Simulator::Now().GetSeconds();
    
    // Rastrear estado de lluvia al momento del envío
    if (g_isRaining) {
        g_metrics.packetsUnderRain++;
        g_metrics.packetSentInRain[packetUid] = true;
    } else {
        g_metrics.packetsSentNoRain++;
        g_metrics.packetSentInRain[packetUid] = false;
    }
    
    // Determinar cluster basado en la IP del SENSOR (from)
    uint32_t sourceCluster = 999;
    uint32_t lastOctet = sensorAddr.Get() & 0xFF;
    
    if (lastOctet > 0 && lastOctet <= 100) {
        uint32_t sensorIndex = lastOctet - 1; // IP .1 = sensor 0
        sourceCluster = sensorIndex / 3; // 0,1,2→0; 3,4,5→1; 6,7,8→2
        
        if (sourceCluster < 3) {
            g_metrics.clusterMetrics[sourceCluster].packetsSent++;
            if (g_isRaining) {
                g_metrics.clusterMetrics[sourceCluster].packetsUnderRain++;
            }
        } else {
            sourceCluster = 999;
        }
    }
    
    g_metrics.packetSourceCluster[packetUid] = sourceCluster;

    // Creamos el payload con la IP del sensor como destino final
    std::ostringstream oss;
    oss << sensorAddr << "|" << msg << "|UID:" << packetUid << "|SRC_CLUSTER:" << sourceCluster;
    std::string payload = oss.str();

    // Crear paquete con ese contenido
    Ptr<Packet> packet = Create<Packet>((uint8_t*)payload.c_str(), payload.size());

    // Crear un socket UDP temporal
    Ptr<Socket> socket = Socket::CreateSocket(from, UdpSocketFactory::GetTypeId());
    socket->Connect(InetSocketAddress(chAddr, port));
    socket->Send(packet);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Sensor "
                    << from->GetId()
                    << " envía STATUS: \"" << msg << "\" a CH (finalDest="
                    << sensorAddr << ") [UID=" << packetUid << ", Cluster=" << sourceCluster << "]");
}

void SetupIntraClusterClients(NodeContainer &sensors,
                              NodeContainer &clusterHeads,
                              Ipv4InterfaceContainer &interfaces,
                              double simTime)
{

    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        // Calcular el Cluster Head correspondiente (3 sensores por cluster)
        uint32_t headIndex = i / 3;
        Ptr<Node> headNode = clusterHeads.Get(headIndex);   // ahora usamos el nodo
        Ipv4Address headAddr = interfaces.GetAddress(9 + headIndex); // IP del CH

        double startTime = 2.0 + i * 0.2; // escalonamiento
        double interval  = 10.0;  // Enviar cada 10 segundos
        double stopTime  = simTime - 2.0;  // Hasta 2 segundos antes del fin

        // Programar envíos periódicos de Status/Event al Cluster Head
        for (double t = startTime; t <= stopTime; t += interval)
        {
            Simulator::Schedule(Seconds(t), [=]() {
                    SendStatusEvent(sensors.Get(i), headAddr, headAddr,
                                    "Reporte periódico al CH " + std::to_string(headIndex));
                });

        }

    }
}


void ClusterHeadAppRecvCallback(uint32_t headIndex, Ptr<Node> chNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    std::vector<uint8_t> buffer(pktSize);
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    
    // Separar IP destino y resto del mensaje
    size_t sep1 = payload.find('|');
    if (sep1 == std::string::npos)
    {
        NS_LOG_WARN("CH[" << headIndex << "] paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep1);
    std::string rest = payload.substr(sep1 + 1);

    bool isResend = false;
    std::string msg;

    // Buscar si tiene el flag RESEND al final
    size_t sep2 = rest.rfind('|');
    if (sep2 != std::string::npos)
    {
        std::string flag = rest.substr(sep2 + 1);
        if (flag == "RESEND")
        {
            isResend = true;
            msg = rest.substr(0, sep2);
        }
        else
        {
            msg = rest; // paquete original sin flag
        }
    }
    else
    {
        msg = rest; // paquete original sin flag
    }

    Ipv4Address destIp(ipStr.c_str());

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] recibió paquete con destino: " << destIp 
                    << (isResend ? " [RESEND]" : "")
                    << " | Mensaje: " << msg);

    // Verificar si el destino está en el mismo cluster
    bool inCluster = false;
    NodeContainer clusterNodes = g_clusterNodesMap[chNode->GetId()];
    for (uint32_t i = 0; i < clusterNodes.GetN(); ++i)
    {
        Ptr<Node> n = clusterNodes.Get(i);
        Ptr<Ipv4> ipv4 = n->GetObject<Ipv4>();
        if (!ipv4) continue;

        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j)
        {
            Ipv4Address addr = ipv4->GetAddress(j, 0).GetLocal();
            if (addr == destIp)
            {
                inCluster = true;
                break;
            }
        }
        if (inCluster) break;
    }

    // Si destino es el mismo CH, redirigir al supercluster
    Ptr<Ipv4> chIpv4 = chNode->GetObject<Ipv4>();
    if (chIpv4 && destIp == chIpv4->GetAddress(1,0).GetLocal()) destIp = g_superAddr;

    Ptr<Packet> safeCopy = packet->Copy();

    if (inCluster)
    {
        // Enviar paquete directo (tanto original como RESEND)
        Ptr<Socket> socket = Socket::CreateSocket(chNode, UdpSocketFactory::GetTypeId());
        socket->Connect(InetSocketAddress(destIp, 4000));
        Ptr<Packet> newPacket = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        socket->Send(newPacket);
        NS_LOG_INFO("CH[" << headIndex << "] envió mensaje directo a " << destIp
                        << (isResend ? " [RESEND]" : " [ORIGINAL]"));
    }
    else
    {
        // Si no está en el cluster, bufferizar según si es RESEND o paquete original
        if (isResend)
        {
            BufferPacketSent(chNode, destIp, msg, safeCopy);
            NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] bufferizó paquete REENVÍO para " << destIp);
        }
        else
        {
            BufferPacket(chNode, destIp, msg, safeCopy);
            NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] bufferizó paquete para " << destIp);
        }
    }

}


void SensorAppRecvCallback(uint32_t nodeIndex, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    uint8_t buffer[1024];
    packet->CopyData(buffer, pktSize);
    std::string msg(reinterpret_cast<char*>(buffer), pktSize);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] Sensor[" << nodeIndex << "] recibió mensaje: "
                    << msg);
}

void RecolectorAppRecvCallback(uint32_t recolectorIndex, Ptr<Node> recolectorNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    std::vector<uint8_t> buffer(pktSize);   
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    

    // Separar IP destino y mensaje
    size_t sep1 = payload.find('|');
    if (sep1 == std::string::npos)
    {
        NS_LOG_WARN("Recolector[" << recolectorIndex << "] recibió paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep1);
    std::string rest = payload.substr(sep1 + 1);

    bool isResend = false;
    std::string msg;

    size_t sep2 = rest.rfind('|'); // buscar último '|'
    if (sep2 != std::string::npos)
    {
        std::string flag = rest.substr(sep2 + 1);
        if (flag == "RESEND")
        {
            isResend = true;
            msg = rest.substr(0, sep2);
        }
        else
        {
            msg = rest; // paquete original sin flag
        }
    }
    else
    {
        msg = rest; // paquete original sin flag
    }

    //add log (improve log with ip and message separated)
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] Recolector[" << recolectorIndex << "] recibió paquete para "
                    << ipStr << (isResend ? " [RESEND]" : "") 
                    << " | Mensaje: " << msg);

    Ipv4Address destIp(ipStr.c_str());

    Ptr<Packet> safeCopy = packet->Copy();
    if (isResend)
    {
        BufferPacketSent(recolectorNode, destIp, msg, safeCopy);
        //add sim time

        NS_LOG_INFO("["<<Simulator::Now().GetSeconds()<<"s] Recolector[" << recolectorIndex << "] bufferizó paquete REENVÍO para " << destIp);
    }
    else
    {
        BufferPacket(recolectorNode, destIp, msg, safeCopy);
        NS_LOG_INFO("["<<Simulator::Now().GetSeconds()<<"s]Recolector[" << recolectorIndex << "] bufferizó paquete para " << destIp);
    }
}



// Callback para el supercluster
void SuperClusterAppRecvCallback(uint32_t scIndex, Ptr<Node> scNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();

    std::vector<uint8_t> buffer(pktSize);
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    //  Registrar entrega exitosa al SuperCluster
    g_metrics.packetsDeliveredToSuper++;
    
    // Extraer UID para análisis detallado
    uint32_t packetUid = 0;
    uint32_t sourceCluster = 999;
    bool wasRain = false;
    
    size_t uidPos = payload.find("|UID:");
    if (uidPos != std::string::npos) {
        std::string uidStr = payload.substr(uidPos + 5);
        size_t endUid = uidStr.find('|');
        if (endUid != std::string::npos) {
            uidStr = uidStr.substr(0, endUid);
        }
        
        try {
            packetUid = std::stoul(uidStr);
            
            // CORREGIDO: Verificar si fue enviado bajo lluvia (no el estado actual)
            if (g_metrics.packetSentInRain.find(packetUid) != g_metrics.packetSentInRain.end()) {
                wasRain = g_metrics.packetSentInRain[packetUid];
                if (wasRain) {
                    g_metrics.deliveredUnderRain++;
                } else {
                    g_metrics.deliveredNoRain++;
                }
            }
            
            // Calcular latencia
            if (g_metrics.packetSendTime.find(packetUid) != g_metrics.packetSendTime.end()) {
                double sendTime = g_metrics.packetSendTime[packetUid];
                double latency = Simulator::Now().GetSeconds() - sendTime;
                
                // Latencia global
                g_metrics.latencies.push_back(latency);
                g_metrics.totalLatency += latency;
                if (latency < g_metrics.minLatency) g_metrics.minLatency = latency;
                if (latency > g_metrics.maxLatency) g_metrics.maxLatency = latency;
                
                // Latencia por cluster
                if (g_metrics.packetSourceCluster.find(packetUid) != g_metrics.packetSourceCluster.end()) {
                    sourceCluster = g_metrics.packetSourceCluster[packetUid];
                    if (sourceCluster < 3) {
                        g_metrics.clusterMetrics[sourceCluster].packetsDelivered++;
                        g_metrics.clusterMetrics[sourceCluster].latencies.push_back(latency);
                        g_metrics.clusterMetrics[sourceCluster].totalLatency += latency;
                        
                        if (wasRain) {
                            g_metrics.clusterMetrics[sourceCluster].deliveredUnderRain++;
                        }
                    }
                }
            }
        } catch (...) {
            // Ignorar errores de conversión
        }
    }
    
    // Extraer cluster origen para métricas inter-cluster
    size_t srcPos = payload.find("|SRC_CLUSTER:");
    if (srcPos != std::string::npos) {
        std::string srcStr = payload.substr(srcPos + 13);
        size_t endSrc = srcStr.find('|');
        if (endSrc != std::string::npos) {
            srcStr = srcStr.substr(0, endSrc);
        }
        
        try {
            sourceCluster = std::stoul(srcStr);
        } catch (...) {
            // Ignorar
        }
    }

    // Separar IP destino y mensaje
    size_t sep = payload.find('|');

    //update log with destination ip and message
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] SuperCluster[" << scIndex << "] recibió paquete para "
                    << payload.substr(0, sep) << " | Mensaje: " 
                    << payload.substr(sep + 1));
    
    if (sep == std::string::npos)
    {
        NS_LOG_WARN("SuperCluster[" << scIndex << "] recibió paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep);
    std::string msg = payload.substr(sep + 1); 

    //setear destip
    Ipv4Address destIp(ipStr.c_str());

    //discard if the message is intended for the supercluster itself
    Ptr<Ipv4> scIpv4 = scNode->GetObject<Ipv4>();
    if (scIpv4 && destIp == scIpv4->GetAddress(1,0).GetLocal()) 
    {
        NS_LOG_INFO("SuperCluster[" << scIndex << "] recibió mensaje destinado a sí mismo: " << msg);
        return;
    }

    //  Marcar el payload como RESEND antes de bufferizar
    msg = msg + "|RESEND";
    std::string resendPayload = ipStr + "|" + msg + "|RESEND";
    Ptr<Packet> newPkt = Create<Packet>(reinterpret_cast<const uint8_t*>(resendPayload.c_str()), resendPayload.size());

    // Bufferizar el paquete completo en el buffer de reenvío
    BufferPacketSent(scNode, destIp, msg, newPkt);

    NS_LOG_INFO("SuperCluster[" << scIndex << "] bufferizó mensaje para REENVÍO a " << destIp);
}

void InstallUdpServers(NodeContainer &sensors, NodeContainer &recolectors, NodeContainer &superCluster, NodeContainer &clusterHeads, double simTime)
{
    // 1️⃣ Server en cada recolector (recibe datos de clusters)
    {
        uint16_t port = 5000;
        UdpServerHelper recolectorServer(port);

        for (uint32_t i = 0; i < recolectors.GetN(); ++i)
        {
            ApplicationContainer apps = recolectorServer.Install(recolectors.Get(i));
            apps.Start(Seconds(1.0));

            // Obtener la instancia del servidor UDP recién instalado
            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

            // Conectar la traza Rx con el callback, pasando el ID del recolector
            srv->TraceConnectWithoutContext("Rx",
                MakeBoundCallback(&RecolectorAppRecvCallback, i, recolectors.Get(i)));

            NS_LOG_INFO("Servidor Recolector " << i
                        << " instalado en nodo " << recolectors.Get(i)->GetId()
                        << " (puerto " << port << ")");
        }

    }

    // 2️⃣ Server en el supercluster (recibe uploads de drones)
    {
        uint16_t port = 6000;
        UdpServerHelper server(port);
        ApplicationContainer apps = server.Install(superCluster.Get(0));
        apps.Start(Seconds(1.0));

        // Obtener la instancia del servidor UDP recién instalado
        Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

        // Conectar la traza Rx con el callback, pasando el índice 0 (único nodo del supercluster)
        srv->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SuperClusterAppRecvCallback, 0, superCluster.Get(0)));

        NS_LOG_INFO("Servidor SuperCluster instalado en nodo " 
                    << superCluster.Get(0)->GetId() 
                    << " (puerto " << port << ")");

    }


    //  Servidor UDP interno en cada cluster head (para recibir de sensores)
    {
        uint16_t intraPort = 4000;
        UdpServerHelper intraServer(intraPort);

        for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
        {
            Ptr<Node> chNode = clusterHeads.Get(i);  //  obtenemos el puntero del Cluster Head

            ApplicationContainer apps = intraServer.Install(chNode);
            apps.Start(Seconds(0.5));

            //  Obtiene la instancia del servidor UDP recién instalado
            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

            //  Conecta la traza Rx con tu callback, pasando también el nodo
            srv->TraceConnectWithoutContext("Rx",
                MakeBoundCallback(&ClusterHeadAppRecvCallback, i, chNode));

            NS_LOG_INFO("Servidor interno CH " << i
                        << " instalado en nodo " << chNode->GetId()
                        << " (puerto " << intraPort << ")");
        }
    }


    {
        uint16_t sensorPort = 4000; // Mismo puerto que usan los CH al reenviar
        UdpServerHelper sensorServer(sensorPort);

        for (uint32_t i = 0; i < sensors.GetN(); ++i)
        {
            ApplicationContainer apps = sensorServer.Install(sensors.Get(i));
            apps.Start(Seconds(1.0));

            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));
            srv->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SensorAppRecvCallback, i));

            NS_LOG_INFO("Servidor sensor " << i << " (puerto " << sensorPort << ")");
        }
    }
}

void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C,
                                double speed,
                                double startTime,
                                Vector centroid)
{
    auto computeVelocity = [speed](const Vector &from, const Vector &to) {
        Vector dir = to - from;
        double len = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
        if (len == 0) return Vector(0,0,0);
        return Vector(dir.x / len * speed, dir.y / len * speed, dir.z / len * speed);
    };

    Vector start = mv->GetPosition();
    std::vector<Vector> route = {A, B, C}; // O rotación según inicio si quieres

    double t = startTime;
    Vector from = start;

    for (size_t i=0; i<route.size(); ++i) {
        Vector to = route[i];
        double dist = std::sqrt((to.x - from.x)*(to.x - from.x) + (to.y - from.y)*(to.y - from.y) + (to.z - from.z)*(to.z - from.z));
        double travelTime = dist / speed;
        Vector vel = computeVelocity(from, to);

        Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, vel);
        Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, to);

        t += travelTime;
        from = to;
    }

    // Finalmente, regresar al centroide suavemente
    Vector pos = route.back();
    double distToCentroid = std::sqrt((centroid.x - pos.x)*(centroid.x - pos.x) + (centroid.y - pos.y)*(centroid.y - pos.y) + (centroid.z - pos.z)*(centroid.z - pos.z));
    double travelTime = distToCentroid / speed;
    Vector velToCentroid = computeVelocity(pos, centroid);

    Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, velToCentroid);
    Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);

    // Cuando llegue, reinicia patrulla
    Simulator::Schedule(Seconds(t + travelTime), [=]() {
        ScheduleRecolectorMovement(mv, A, B, C, speed, Simulator::Now().GetSeconds(), centroid);
    });
}


// Calcula y aplica velocidad para que mv llegue a centroid justo al tiempo simEnd.
// Si startLead > 0, programa el cálculo en simEnd - startLead; si startLead == 0 calcula ahora.
static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double patrolSpeed)
{
    Vector pos = mv->GetPosition();
    Vector dir = centroid - pos;
    double dist = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);

    if (dist <= 1e-9) {
        // Llegó al supercluster: reiniciar ruta
        ScheduleRecolectorMovement(mv, Vector(0,0,0), Vector(100,0,0), Vector(50,86.6,0), patrolSpeed, simEnd,centroid);
        return;
    }

    double now = Simulator::Now().GetSeconds();
    double remaining = simEnd - now;
    double speed = (remaining > 0.0) ? dist / remaining : 0.0;
    Vector vel = Vector(dir.x / dist * speed, dir.y / dist * speed, dir.z / dist * speed);

    mv->SetVelocity(vel);
    Simulator::Schedule(Seconds(dist / speed), &ReturnToSuperCallback, mv, centroid, simEnd, patrolSpeed);
}


static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double leadTime, double patrolSpeed)
{
    double start = simEnd - leadTime;
    if (start < 0.0) start = 0.0;

    Simulator::Schedule(Seconds(start), &ReturnToSuperCallback, mv, centroid, simEnd, patrolSpeed);

    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetVelocity, mv, Vector(0,0,0));
}

void PeriodicProximityCheck(NodeContainer &clusterHeads,
                            NodeContainer &recolectors,
                            double checkInterval)
{
    double ptDbm = 0.0;            
    double rxSensitivityDbm = -92; // Mejorado de -85 a -92 dBm (más sensible)
    double txGainDb = 0.0;
    double rxGainDb = 0.0;

    Ptr<MultiZonePropagationLossModel> multiZone = g_multiZoneModel;
    if (!multiZone) {
        NS_LOG_WARN("MultiZone model no inicializado. Abortando chequeo.");
        return;
    }

    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        Ptr<Node> ch = clusterHeads.Get(i);
        Ptr<MobilityModel> mobCH = ch->GetObject<MobilityModel>();
        Vector posCH = mobCH->GetPosition();

        // Obtener los nodos que pertenecen a este cluster head
        auto &clusterNodes = g_clusterNodesMap[ch->GetId()];

        for (uint32_t j = 0; j < recolectors.GetN(); ++j)
        {
            Ptr<Node> recolector = recolectors.Get(j);
            Ptr<MobilityModel> mobR = recolector->GetObject<MobilityModel>();
            Vector posR = mobR->GetPosition();

            double dx = posCH.x - posR.x;
            double dy = posCH.y - posR.y;
            double dz = posCH.z - posR.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            double lossDb = multiZone->GetLoss(posCH, posR);
            double prDbm = ptDbm + txGainDb + rxGainDb - lossDb;

            bool canSend = prDbm >= rxSensitivityDbm;

            //  Registrar chequeo de conectividad
            g_metrics.connectivityChecks++;
            if (canSend) {
                g_metrics.successfulConnections++;
            }

            if (canSend)
            {
                NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                << " puede enviar a Recolector " << j
                                << " (Dist=" << distance << " m, PL=" << lossDb
                                << " dB, Pr=" << prDbm << " dBm)");

                // Direccion IP del recolector
                Ptr<Ipv4> ipv4 = recolector->GetObject<Ipv4>();
                Ipv4Address recolectorAddr = ipv4->GetAddress(1,0).GetLocal();

                // 1️⃣ Enviar paquetes bufferizados del CH
                while (HasBufferedPackets(ch)) {
                    BufferedPacket bpkt = PeekBufferedPacket(ch);
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                    << " enviando paquete bufferizado a Recolector " << j
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    
                    SendSensorData(ch, recolectorAddr, 5000, bpkt.dest, bpkt.msg);
                    PopBufferedPacket(ch);
                }

                // 2️⃣ Revisar buffer de paquetes enviados por el recolector para reenvío
                while (HasBufferedPacketsSent(recolector)) {

                    //ADD LOG
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                    << " revisando paquete bufferizado para reenvío al CH " << i);

                    BufferedPacket bpkt = PeekBufferedPacketSent(recolector);

                    // Solo enviar si el destino pertenece al cluster del CH
                    bool belongsToCluster = false;
                    for (uint32_t k = 0; k < clusterNodes.GetN(); ++k) {
                        Ptr<Node> node = clusterNodes.Get(k);
                        Ptr<Ipv4> ipv4Node = node->GetObject<Ipv4>();
                        if (ipv4Node && bpkt.dest == ipv4Node->GetAddress(1,0).GetLocal()) {
                            belongsToCluster = true;
                            break;
                        }
                    }

                    if (belongsToCluster) {

                        //get ch ip
                        Ptr<Ipv4> chIpv4 = ch->GetObject<Ipv4>();
                        Ipv4Address chAddr = chIpv4->GetAddress(1,0).GetLocal();
                        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                        << " reenviando paquete a CH " << i
                                        << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");
                        SendSensorData(recolector, chAddr, 4000, bpkt.dest, bpkt.msg);
                        PopBufferedPacketSent(recolector);
                    } else {
                        // No pertenece a este cluster, lo dejamos para otro CH
                        break;
                    }
                }
            }
            else
            {
                NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                << " NO puede enviar a Recolector " << j
                                << " (Dist=" << distance << " m, PL=" << lossDb
                                << " dB, Pr=" << prDbm << " dBm)");
            }
        }
    }

    Simulator::Schedule(Seconds(checkInterval),
                        &PeriodicProximityCheck,
                        std::ref(clusterHeads),
                        std::ref(recolectors),
                        checkInterval);
}



void SchedulePeriodicProximityCheck(NodeContainer &clusterHeads,
                                    NodeContainer &recolectors,
                                    double checkInterval)
{
    Simulator::Schedule(Seconds(checkInterval), [=, &clusterHeads, &recolectors]() {
        PeriodicProximityCheck(clusterHeads, recolectors, checkInterval);
    });
}

void SuperClusterProximityCheck(NodeContainer &superCluster,
                                NodeContainer &recolectors,
                                
                                double checkInterval)
{
    // Asumimos TxPower y sensibilidad fijas
    double txPowerDbm = 0.0;         // dBm
    double rxSensitivityDbm = -92.0; // Mejorado de -85 a -92 dBm (más sensible)

    Simulator::Schedule(Seconds(checkInterval), &SuperClusterProximityCheck,
                        superCluster, recolectors, checkInterval);

    for (uint32_t i = 0; i < superCluster.GetN(); ++i)
    {
        Ptr<Node> scNode = superCluster.Get(i);
        Ptr<MobilityModel> scMob = scNode->GetObject<MobilityModel>();
        Vector scPos = scMob->GetPosition();

        
                

        for (uint32_t j = 0; j < recolectors.GetN(); ++j)
        {
            Ptr<Node> rcNode = recolectors.Get(j);
            Ptr<MobilityModel> rcMob = rcNode->GetObject<MobilityModel>();
            Vector rcPos = rcMob->GetPosition();

            // Calcular pérdida usando MultiZonePropagationLossModel global
            double pathLoss = g_multiZoneModel->GetLoss(rcPos, scPos); // rx = SC, tx = Recolector
            double rxPower = txPowerDbm - pathLoss;

            //get recolector address
            Ptr<Node> node = recolectors.Get(j);
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
            Ipv4Address recolectorAddr = ipv4->GetAddress(1,0).GetLocal();

            if (rxPower >= rxSensitivityDbm)
            {
                NS_LOG_INFO(Simulator::Now().GetSeconds() << "s] Recolector " << j
                    << " puede enviar a SuperCluster " << i
                    << " (Dist=" << CalculateDistance(rcPos, scPos)
                    << " m, PL=" << pathLoss
                    << " dB, Pr=" << rxPower << " dBm)");
                
                while (HasBufferedPackets(rcNode)) {
                    BufferedPacket bpkt = PeekBufferedPacket(rcNode);

                    //get supercluster ip
                    Ptr<Ipv4> scIpv4 = scNode->GetObject<Ipv4>();
                    Ipv4Address scAddr = scIpv4->GetAddress(1,0).GetLocal();
                    
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                    << " enviando paquete bufferizado a SuperCluster " << i
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    SendSensorData(rcNode, scAddr , 6000 , bpkt.dest, bpkt.msg );

                    // Eliminar paquete de la cola
                    PopBufferedPacket(rcNode);
                }


                while (HasBufferedPacketsSent(scNode)) {
                    BufferedPacket bpkt = PeekBufferedPacketSent(scNode);
                    
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] SuperCluster " << i
                                    << " enviando paquete bufferizado al Recolector " << j
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    
                    SendSensorData(scNode, recolectorAddr , 5000 , bpkt.dest, bpkt.msg );

                    // Eliminar paquete de la cola
                    PopBufferedPacketSent(scNode);

                }
                

                
            }
            else
            {
                NS_LOG_INFO(Simulator::Now().GetSeconds() << "s] Recolector " << j
                    << " NO puede enviar a SuperCluster " << i
                    << " (Dist=" << CalculateDistance(rcPos, scPos)
                    << " m, PL=" << pathLoss
                    << " dB, Pr=" << rxPower << " dBm)");
            }
        }
    }
}



void ScheduleSuperClusterProximityCheck(NodeContainer &superCluster,
                                        NodeContainer &recolectors,
                                        double checkInterval)
{
    Simulator::Schedule(Seconds(checkInterval), [=, &superCluster, &recolectors]() {
        SuperClusterProximityCheck(superCluster, recolectors, checkInterval);
    });
}

void ScheduleSendToOtherCluster(Ptr<Node> srcNode, Ipv4Address destIp, uint16_t port, double sizeBytes, double delaySeconds) {
    Simulator::Schedule(Seconds(delaySeconds), [=]() {
        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Nodo " 
                        << srcNode->GetId() 
                        << " preparando envío de paquete a nodo con IP " << destIp);

        // Crear el paquete
        Ptr<Packet> pkt = Create<Packet>(static_cast<uint32_t>(sizeBytes));

        // -----------------------------
        // Buscar el cluster head del nodo emisor
        // -----------------------------
        Ipv4Address clusterHeadIp;
        bool found = false;
        for (auto &pair : g_clusterNodesMap) {
            uint32_t chId = pair.first;
            NodeContainer nodes = pair.second;

            for (uint32_t i = 0; i < nodes.GetN(); ++i) {
                if (nodes.Get(i)->GetId() == srcNode->GetId()) {
                    Ptr<Ipv4> ipv4 = NodeList::GetNode(chId)->GetObject<Ipv4>();
                    clusterHeadIp = ipv4->GetAddress(1,0).GetLocal(); // interfaz ad-hoc
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        if (!found) {
            NS_LOG_WARN("Nodo " << srcNode->GetId() << " no pertenece a ningún cluster head, usando destIp directamente");
            clusterHeadIp = destIp;
        }

        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                        << srcNode->GetId() 
                        << " enviará paquete primero al ClusterHead con IP " << clusterHeadIp);

        // Bufferizar el paquete hacia el cluster head, indicando el destino final
        SendStatusEvent(srcNode, clusterHeadIp, destIp, "Saludos desde el nodo"+std::to_string(srcNode->GetId())+"!");
    });
}



void RunSimulation(double simTime, bool enableRain)
{
    //  Aumentar período de lluvia para capturar más paquetes
    double startRain = 60.0;   // Inicio de lluvia (1 minuto)
    double endRain   = 180.0;  // Fin de lluvia (3 minutos) - 2 minutos de lluvia

    if (g_multiZoneModel && enableRain)
    {
        Simulator::Schedule(Seconds(startRain), [=]() {
            g_multiZoneModel->SetRainEffect(true);
            g_isRaining = true; //  Actualizar estado global
            g_metrics.timeUnderRain = endRain - startRain;
            NS_LOG_INFO(" Lluvia activada en t=" << startRain << "s (duración: " << (endRain - startRain) << "s)");
        });

        Simulator::Schedule(Seconds(endRain), [=]() {
            g_multiZoneModel->SetRainEffect(false);
            g_isRaining = false; //  Actualizar estado global
            NS_LOG_INFO("☀️ Lluvia desactivada en t=" << endRain << "s");
        });
    }
    else
    {
        if (!g_multiZoneModel) {
            NS_LOG_WARN("g_multiZoneModel is null: CreateMultiZoneChannel() debe llamarse antes de RunSimulation()");
        } else {
            NS_LOG_INFO("☀️ Simulación SIN lluvia (enableRain=false)");
        }
    }

    

    Simulator::Stop(Seconds(simTime));
    
    Simulator::Run();
    Simulator::Destroy();
}

// ---------------------------
//  FUNCIONES DE ANÁLISIS DE MÉTRICAS
// ---------------------------

void PrintMetricsReport(const std::string& scenarioName, double simTime, uint32_t nDrones)
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       REPORTE DE MÉTRICAS - " << scenarioName << std::setw(35 - scenarioName.length()) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Configuración de simulación
    std::cout << "║ CONFIGURACIÓN:                                               ║\n";
    std::cout << "║   • Tiempo de simulación: " << std::fixed << std::setprecision(1) 
              << std::setw(10) << simTime << " s" << std::setw(23) << "║\n";
    std::cout << "║   • Número de drones: " << std::setw(10) << nDrones << std::setw(30) << "║\n";
    std::cout << "║   • Tiempo bajo lluvia: " << std::setw(10) << g_metrics.timeUnderRain 
              << " s" << std::setw(23) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Métricas globales de paquetes
    std::cout << "║ ENTREGA DE PAQUETES (GLOBAL):                                ║\n";
    std::cout << "║   • Paquetes enviados: " << std::setw(10) << g_metrics.totalPacketsSent 
              << std::setw(31) << "║\n";
    std::cout << "║   • Entregados al Super: " << std::setw(10) << g_metrics.packetsDeliveredToSuper 
              << std::setw(29) << "║\n";
    std::cout << "║   • Paquetes perdidos: " << std::setw(10) << g_metrics.packetsLost
              << std::setw(31) << "║\n";
    std::cout << "║   • PDR (% entrega): " << std::setw(10) << std::setprecision(2) 
              << g_metrics.GetPDR() << " %" << std::setw(27) << "║\n";
    std::cout << "║   • Tasa de pérdida: " << std::setw(10) << std::setprecision(2)
              << g_metrics.GetPacketLossRate() << " %" << std::setw(27) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Latencia global
    std::cout << "║ LATENCIA (GLOBAL):                                           ║\n";
    std::cout << "║   • Latencia promedio: " << std::setw(10) << std::setprecision(3) 
              << g_metrics.GetAverageLatency() << " s" << std::setw(25) << "║\n";
    std::cout << "║   • Latencia mínima: " << std::setw(10) << std::setprecision(3)
              << (g_metrics.minLatency == std::numeric_limits<double>::max() ? 0.0 : g_metrics.minLatency)
              << " s" << std::setw(27) << "║\n";
    std::cout << "║   • Latencia máxima: " << std::setw(10) << std::setprecision(3)
              << g_metrics.maxLatency << " s" << std::setw(27) << "║\n";
    std::cout << "║   • Desviación estándar: " << std::setw(10) << std::setprecision(3)
              << g_metrics.GetLatencyStdDev() << " s" << std::setw(25) << "║\n";
    std::cout << "║   • Muestras de latencia: " << std::setw(10) << g_metrics.latencies.size() 
              << std::setw(29) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Métricas por cluster
    std::cout << "║ MÉTRICAS POR CLUSTER:                                        ║\n";
    const char* clusterNames[] = {"A", "B", "C"};
    for (uint32_t i = 0; i < 3; ++i) {
        if (g_metrics.clusterMetrics.find(i) != g_metrics.clusterMetrics.end()) {
            const ClusterMetrics& cm = g_metrics.clusterMetrics[i];
            std::cout << "║    CLUSTER " << clusterNames[i] << ":                                            ║\n";
            std::cout << "║     - Enviados: " << std::setw(6) << cm.packetsSent
                      << " | Entregados: " << std::setw(6) << cm.packetsDelivered
                      << " | PDR: " << std::setw(5) << std::setprecision(1) << cm.GetPDR() << "%" << std::setw(8) << "║\n";
            std::cout << "║     - Lat. Prom: " << std::setw(6) << std::setprecision(3) << cm.GetAvgLatency()
                      << "s | Pérdida: " << std::setw(5) << std::setprecision(1) << cm.GetPacketLossRate() << "%" << std::setw(18) << "║\n";
        }
    }
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Conectividad
    std::cout << "║ CONECTIVIDAD:                                                ║\n";
    std::cout << "║   • Chequeos totales: " << std::setw(10) << g_metrics.connectivityChecks 
              << std::setw(32) << "║\n";
    std::cout << "║   • Conexiones exitosas: " << std::setw(10) << g_metrics.successfulConnections 
              << std::setw(29) << "║\n";
    std::cout << "║   • Tasa de conectividad: " << std::setw(10) << std::setprecision(2) 
              << g_metrics.GetConnectivityRate() << " %" << std::setw(25) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Impacto de lluvia (CORREGIDO)
    std::cout << "║ IMPACTO DE LLUVIA:                                           ║\n";
    std::cout << "║   • Paquetes enviados CON lluvia: " << std::setw(10) 
              << g_metrics.packetsUnderRain << std::setw(21) << "║\n";
    std::cout << "║   • Entregados (de los CON lluvia): " << std::setw(10) 
              << g_metrics.deliveredUnderRain << std::setw(19) << "║\n";
    std::cout << "║   • PDR con lluvia: " << std::setw(10) << std::setprecision(2) 
              << g_metrics.GetRainPDR() << " %" << std::setw(28) << "║\n";
    std::cout << "║   • Paquetes enviados SIN lluvia: " << std::setw(10)
              << g_metrics.packetsSentNoRain << std::setw(21) << "║\n";
    std::cout << "║   • Entregados (de los SIN lluvia): " << std::setw(10)
              << g_metrics.deliveredNoRain << std::setw(19) << "║\n";
    std::cout << "║   • PDR sin lluvia: " << std::setw(10) << std::setprecision(2)
              << g_metrics.GetNoRainPDR() << " %" << std::setw(26) << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Drones
    std::cout << "║ ACTIVIDAD DE DRONES:                                         ║\n";
    std::cout << "║   • Alarmas generadas: " << std::setw(10) << g_metrics.alarmsTriggers 
              << std::setw(31) << "║\n";
    std::cout << "║   • Intercepciones de drones: " << std::setw(10) 
              << g_metrics.droneInterceptions << std::setw(25) << "║\n";
    if (g_metrics.alarmsTriggers > 0) {
        double efficiency = (100.0 * g_metrics.droneInterceptions) / g_metrics.alarmsTriggers;
        std::cout << "║   • Eficiencia de respuesta: " << std::setw(10) << std::setprecision(2)
                  << efficiency << " %" << std::setw(24) << "║\n";
    }
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
}
void ExportMetricsToCSV(const std::string& filename, const std::string& scenarioName, 
                        double simTime, uint32_t nDrones)
{
    std::ofstream csvFile;
    bool fileExists = std::ifstream(filename).good();
    
    csvFile.open(filename, std::ios::app);
    
    if (!fileExists) {
        // Escribir encabezados mejorados
        csvFile << "Escenario,Tiempo_Sim,Num_Drones,"
                << "Paq_Enviados,Paq_Entregados,Paq_Perdidos,PDR_%,Tasa_Perdida_%,"
                << "Lat_Prom_s,Lat_Min_s,Lat_Max_s,Lat_StdDev_s,"
                << "Conectividad_%,"
                << "Paq_Con_Lluvia,Entregados_Con_Lluvia,PDR_Lluvia_%,"
                << "Paq_Sin_Lluvia,Entregados_Sin_Lluvia,PDR_Sin_Lluvia_%,"
                << "ClusterA_Enviados,ClusterA_Entregados,ClusterA_PDR_%,ClusterA_Lat_s,"
                << "ClusterB_Enviados,ClusterB_Entregados,ClusterB_PDR_%,ClusterB_Lat_s,"
                << "ClusterC_Enviados,ClusterC_Entregados,ClusterC_PDR_%,ClusterC_Lat_s,"
                << "Alarmas,Intercepciones,Eficiencia_Drones_%\n";
    }
    
    // Obtener métricas por cluster
    const ClusterMetrics& cmA = g_metrics.clusterMetrics[0];
    const ClusterMetrics& cmB = g_metrics.clusterMetrics[1];
    const ClusterMetrics& cmC = g_metrics.clusterMetrics[2];
    
    // Calcular paquetes perdidos (enviados - entregados)
    g_metrics.packetsLost = g_metrics.totalPacketsSent > g_metrics.packetsDeliveredToSuper 
        ? g_metrics.totalPacketsSent - g_metrics.packetsDeliveredToSuper 
        : 0;
    
    double droneEfficiency = g_metrics.alarmsTriggers > 0 ? 
        (100.0 * g_metrics.droneInterceptions / g_metrics.alarmsTriggers) : 0.0;
    
    csvFile << scenarioName << ","
            << simTime << ","
            << nDrones << ","
            << g_metrics.totalPacketsSent << ","
            << g_metrics.packetsDeliveredToSuper << ","
            << g_metrics.packetsLost << ","
            << std::fixed << std::setprecision(2) << g_metrics.GetPDR() << ","
            << std::setprecision(2) << g_metrics.GetPacketLossRate() << ","
            << std::setprecision(3) << g_metrics.GetAverageLatency() << ","
            << std::setprecision(3) << (g_metrics.minLatency == std::numeric_limits<double>::max() ? 0.0 : g_metrics.minLatency) << ","
            << std::setprecision(3) << g_metrics.maxLatency << ","
            << std::setprecision(3) << g_metrics.GetLatencyStdDev() << ","
            << std::setprecision(2) << g_metrics.GetConnectivityRate() << ","
            << g_metrics.packetsUnderRain << ","
            << g_metrics.deliveredUnderRain << ","
            << std::setprecision(2) << g_metrics.GetRainPDR() << ","
            << g_metrics.packetsSentNoRain << ","
            << g_metrics.deliveredNoRain << ","
            << std::setprecision(2) << g_metrics.GetNoRainPDR() << ","
            << cmA.packetsSent << ","
            << cmA.packetsDelivered << ","
            << std::setprecision(2) << cmA.GetPDR() << ","
            << std::setprecision(3) << cmA.GetAvgLatency() << ","
            << cmB.packetsSent << ","
            << cmB.packetsDelivered << ","
            << std::setprecision(2) << cmB.GetPDR() << ","
            << std::setprecision(3) << cmB.GetAvgLatency() << ","
            << cmC.packetsSent << ","
            << cmC.packetsDelivered << ","
            << std::setprecision(2) << cmC.GetPDR() << ","
            << std::setprecision(3) << cmC.GetAvgLatency() << ","
            << g_metrics.alarmsTriggers << ","
            << g_metrics.droneInterceptions << ","
            << std::setprecision(2) << droneEfficiency << "\n";
    
    csvFile.close();
    
    std::cout << " Métricas exportadas a: " << filename << "\n";
}

void AnalyzeCoverageArea(NodeContainer& nodes, Vector A, Vector B, Vector C)
{
    // Calcular área del triángulo formado por A, B, C
    double areaABC = 0.5 * std::abs(
        A.x * (B.y - C.y) +
        B.x * (C.y - A.y) +
        C.x * (A.y - B.y)
    );
    
    std::cout << "\n ANÁLISIS DE COBERTURA:\n";
    std::cout << "  • Área total del escenario: " << std::fixed << std::setprecision(2) 
              << areaABC << " m²\n";
    std::cout << "  • Nodos totales en la red: " << nodes.GetN() << "\n";
    std::cout << "  • Densidad de nodos: " << std::setprecision(4) 
              << (nodes.GetN() / areaABC) * 1000.0 << " nodos/km²\n\n";
}

// ---------------------------
//  Main
// ---------------------------
int main(int argc, char *argv[])
{
    // Inicializar el generador aleatorio
    SeedManager::SetSeed(1); 
    g_rand = CreateObject<UniformRandomVariable>();

    NS_LOG_INFO("Simulación iniciando...");

    LogComponentEnable("ManetRecolector", LOG_LEVEL_INFO);

    double simTime = 300.0;  // Tiempo de simulación en segundos
    uint32_t nSensors = 9, nClusterHeads = 3;
    uint32_t nRecolector = 1; // [NUEVO] número de drones por CLI
    bool useLeaderSignalPower = false; // [NUEVO]
    bool enableRain = true; //  Nuevo: activar/desactivar lluvia
    std::string scenarioName = "Base"; //  Nuevo: nombre del escenario

    // [NUEVO] Activable desde línea de comandos
    CommandLine cmd;
    cmd.AddValue("useLeaderSignalPower", "Activa mayor potencia en los líderes de los clusters", useLeaderSignalPower);
    cmd.AddValue("nRecolector", "Número de nodos recolectors (drones)", nRecolector);
    cmd.AddValue("simTime", "Duración de la simulación (s)", simTime);
    cmd.AddValue("enableRain", "Activar efecto de lluvia (true/false)", enableRain);
    cmd.AddValue("scenario", "Nombre del escenario para reportes", scenarioName);
    cmd.Parse(argc, argv);

    NodeContainer sensors, clusterHeads, recolectors, superCluster;
    sensors.Create(nSensors);
    clusterHeads.Create(nClusterHeads);
    recolectors.Create(nRecolector);
    superCluster.Create(1);
    
    // Inicializar métricas de clusters
    for (uint32_t i = 0; i < nClusterHeads; ++i) {
        g_metrics.clusterMetrics[i] = ClusterMetrics();
    }
    NS_LOG_INFO("Métricas de clusters inicializadas para " << nClusterHeads << " clusters");

    // ---------------------------
    // Crear mapa de nodos por cluster
    // ---------------------------
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        NodeContainer clusterNodes;

        // Distribuir sensores entre los cluster heads (simple round-robin)
        for (uint32_t j = 0; j < sensors.GetN(); ++j)
        {
            if (j % clusterHeads.GetN() == i)
            {
                clusterNodes.Add(sensors.Get(j));
            }
        }

        // Guardar en el mapa global (por ID de nodo para callbacks)
        g_clusterNodesMap[clusterHeads.Get(i)->GetId()] = clusterNodes;
        
        // Guardar TAMBIÉN por índice para métricas (0, 1, 2)
        g_clusterByIndexMap[i] = clusterNodes;

        // Logging
        NS_LOG_INFO("ClusterHead " << i << " tiene " << clusterNodes.GetN() << " nodos.");
    }


    // Crear el objeto de animación aquí, al inicio de main()
    AnimationInterface anim("manet_recolector.xml");
    g_anim = &anim; // Asignar el puntero global AHORA

    WifiHelper wifi;
    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    ConfigureWifiAdhoc(wifi, wifiMac, wifiPhy);

    NetDeviceContainer sensorDevices, headDevices, superDevices, recolectorDevices;
    InstallWifiDevices(wifi, wifiMac, wifiPhy, sensors, clusterHeads, superCluster, recolectors,
                       sensorDevices, headDevices, superDevices, recolectorDevices,
                       useLeaderSignalPower); // [NUEVO]

    Vector A(0.0, 0.0, 0.0), B(100.0, 0.0, 0.0), C(50.0, 86.6025403784, 0.0);
    Vector centroid((A.x + B.x + C.x) / 3.0, (A.y + B.y + C.y) / 3.0, 0.0);

    // Almacenar la ruta de patrulla
    std::vector<Vector> patrolRoute = {A, B, C};
    double patrolSpeed = 10.0; // m/s
    double alarmSpeed = 20.0;  // m/s

    ConfigureClusterMobility(sensors, A, B, C);
    MobilityHelper mobility;
    ConfigureClusterHeadsAndSuper(clusterHeads, superCluster, mobility, A, B, C, centroid);

    // Configurar movilidad para múltiples recolectores y obtener sus modelos
    std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs = ConfigureRecolectorMobilities(recolectors, mobility, A, B, C);

    InstallAodvAndStack(sensors, clusterHeads, superCluster, recolectors);

    Ipv4AddressHelper address;
    Ipv4InterfaceContainer interfaces = AssignIpAddresses(address, sensorDevices, headDevices, superDevices, recolectorDevices);

    

    // Obtener direcciones IP de los recolectores
    std::vector<Ipv4Address> recolectorAddrs;
    for (uint32_t i = 0; i < recolectorDevices.GetN(); ++i)
    {
        Ptr<Node> node = recolectors.Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal(); // Interfaz 1 = WiFi Adhoc
        recolectorAddrs.push_back(addr);

        NS_LOG_INFO("Recolector " << i << " -> " << addr);
    }

    // Dirección del supercluster (único nodo)
    g_superAddr = superCluster.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    NS_LOG_INFO("Dirección del supercluster: " << g_superAddr);


    // Instalar aplicaciones UDP (servidores en cada recolectors, clientes en cluster heads -> round-robin)
    InstallUdpServers(sensors, recolectors, superCluster, clusterHeads, simTime);

    // Configurar clientes intra-cluster (sensores -> cluster heads)
    SetupIntraClusterClients(sensors, clusterHeads, interfaces, simTime);

    Ptr<Node> src = sensors.Get(2);
    Ipv4Address destIp = interfaces.GetAddress(7); // dirección IP del nodo destino
    uint16_t port = 6000;
    double packetSize = 100; // bytes
    double delay = 5.0; // segundos después de iniciar la simulación

    ScheduleSendToOtherCluster(src, destIp, port, packetSize, delay);

    // -----------------------------------------------------------------
    //  LÓGICA DE INICIO DE MOVIMIENTO 
    // -----------------------------------------------------------------
    //  LÓGICA DE INICIO DE MOVIMIENTO 
    for (uint32_t i = 0; i < recolectorMVs.size(); ++i) {
        Ptr<ConstantVelocityMobilityModel> mv = recolectorMVs[i];

        // Ahora usamos la nueva función que maneja todo el ciclo
        ScheduleRecolectorMovement(mv, A, B, C, patrolSpeed, simTime, centroid);
    }

    // Programar chequeos periódicos de proximidad (cluster heads -> recolectors)
    SchedulePeriodicProximityCheck(clusterHeads, recolectors);

    // Programar chequeos periódicos de proximidad (supercluster -> recolectors)
    double checkInterval = 1.0; // cada segundo
    ScheduleSuperClusterProximityCheck(superCluster, recolectors, checkInterval);


     // -----------------------------------------------------------------
    //  Colores Iniciales (Todos Iguales) 
    // -----------------------------------------------------------------
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        anim.UpdateNodeColor(sensors.Get(i), 0, 0, 255); // Sensores = Azul
    }
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        anim.UpdateNodeColor(clusterHeads.Get(i), 0, 0, 255); // Heads = Azul
    }
    for (uint32_t i = 0; i < recolectors.GetN(); ++i)
    {
        anim.UpdateNodeColor(recolectors.Get(i), 0, 0, 255); // Drones = Azul
    }
    anim.UpdateNodeColor(superCluster.Get(0), 0, 0, 255); // Super = Azul
    // -----------------------------------------------------------------


    // -----------------------------------------------------------------
    //  LÓGICA DE INICIO DE ALARMA 
    // -----------------------------------------------------------------
    // Programamos la *primera* alarma en un tiempo aleatorio
    double firstAlarmTime = g_rand->GetValue(5.0, 15.0);
    Simulator::Schedule(Seconds(firstAlarmTime), &TriggerRandomAlarm,
                        sensors, recolectors, recolectorMVs,
                        patrolRoute, patrolSpeed, alarmSpeed, simTime);
    

    RunSimulation(simTime, enableRain);

    // ---------------------------
    //  GENERAR REPORTES DE MÉTRICAS
    // ---------------------------
    
    // Imprimir reporte en consola
    PrintMetricsReport(scenarioName, simTime, nRecolector);
    
    // Exportar a CSV
    std::string csvFilename = "metricas_simulacion.csv";
    ExportMetricsToCSV(csvFilename, scenarioName, simTime, nRecolector);
    
    // Análisis de cobertura
    NodeContainer allNodes;
    allNodes.Add(sensors);
    allNodes.Add(clusterHeads);
    allNodes.Add(recolectors);
    allNodes.Add(superCluster);
    AnalyzeCoverageArea(allNodes, A, B, C);
    
    std::cout << "\n Simulación completada exitosamente.\n";
    std::cout << " Para ejecutar diferentes escenarios:\n";
    std::cout << "   • Sin lluvia: ./ns3 run \"scratch/adhoc --enableRain=false --scenario=SinLluvia\"\n";
    std::cout << "   • Más drones: ./ns3 run \"scratch/adhoc --nRecolector=3 --scenario=3Drones\"\n";
    std::cout << "   • Tiempo largo: ./ns3 run \"scratch/adhoc --simTime=600 --scenario=600s\"\n\n";

    return 0;
}
