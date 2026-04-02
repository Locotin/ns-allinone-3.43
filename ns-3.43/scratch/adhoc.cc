#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HierarchicalPrecisionAgriManet");

namespace
{

static std::string
MakeUniformRv(double minValue, double maxValue)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "ns3::UniformRandomVariable[Min=" << minValue << "|Max=" << maxValue << "]";
    return oss.str();
}

static Ptr<RandomRectanglePositionAllocator>
CreateUniformPositionAllocator(const Rectangle& bounds)
{
    Ptr<RandomRectanglePositionAllocator> posAlloc = CreateObject<RandomRectanglePositionAllocator>();
    posAlloc->SetAttribute("X", StringValue(MakeUniformRv(bounds.xMin, bounds.xMax)));
    posAlloc->SetAttribute("Y", StringValue(MakeUniformRv(bounds.yMin, bounds.yMax)));
    return posAlloc;
}

static void
ConfigureLeaderWaypoints(const NodeContainer& leaders,
                         double simTime,
                         const Rectangle& fieldBounds,
                         double leaderSpeed)
{
    const double speed = std::max(0.5, leaderSpeed);
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
            NS_FATAL_ERROR("WaypointMobilityModel no instalado en lider " << i);
        }

        const double laneY = fieldBounds.yMin + laneSpacing * static_cast<double>(i + 1);
        double t = 0.0;
        double x = xMin;

        waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
        while (t + legDuration <= simTime + legDuration)
        {
            t += legDuration;
            x = (x == xMin) ? xMax : xMin;
            waypoint->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
        }
    }
}

static void
InstallFollowerClusterMobility(const NodeContainer& followers,
                               Ptr<MobilityModel> leaderMobility,
                               const Rectangle& localBounds,
                               double followerSpeedMax)
{
    if (followers.GetN() == 0)
    {
        return;
    }

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
    MobilityHelper mobility;
    mobility.SetPositionAllocator(CreateUniformPositionAllocator(fieldBounds));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensors);
}

static uint32_t
FindNearestLeader(const Vector& sensorPosition, const NodeContainer& leaders)
{
    uint32_t bestLeader = 0;
    double bestDistance2 = std::numeric_limits<double>::max();

    for (uint32_t i = 0; i < leaders.GetN(); ++i)
    {
        const Vector leaderPosition = leaders.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const double dx = sensorPosition.x - leaderPosition.x;
        const double dy = sensorPosition.y - leaderPosition.y;
        const double distance2 = dx * dx + dy * dy;

        if (distance2 < bestDistance2)
        {
            bestDistance2 = distance2;
            bestLeader = i;
        }
    }

    return bestLeader;
}

} // namespace

int
main(int argc, char* argv[])
{
    // -------------------------------------------------------------------------
    // Parametros de experimento (primer nivel)
    // -------------------------------------------------------------------------
    // Se modelan 3 clusters de agrobots con movilidad jerarquica:
    // 1 lider fijo por corrida + seguidores con movimiento relativo.
    // Sensores estaticos transmiten UDP periodico hacia el lider mas cercano
    // al inicio de la simulacion.
    //
    // Ejemplo de ejecucion:
    // ./ns3 run "adhoc --agrobotsPerCluster=5 --sensorsPerField=24 --simTime=300"
    // -------------------------------------------------------------------------
    const uint32_t nClusters = 3;
    double simTime = 300.0;
    uint32_t agrobotsPerCluster = 5;
    uint32_t sensorsPerField = 24;
    double fieldWidth = 240.0;
    double fieldHeight = 180.0;
    double leaderSpeed = 2.0;
    double followerSpeedMax = 1.2;
    double clusterBoxWidth = 30.0;
    double clusterBoxHeight = 20.0;
    double sensorInterval = 3.0;
    uint32_t packetSize = 128;
    uint32_t rngRun = 1;
    std::string flowmonFile = "flowmon-adhoc.xml";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Tiempo total de simulacion en segundos", simTime);
    cmd.AddValue("agrobotsPerCluster",
                 "Cantidad de agrobots por cluster (incluye al lider)",
                 agrobotsPerCluster);
    cmd.AddValue("sensorsPerField", "Cantidad de sensores estaticos en el campo", sensorsPerField);
    cmd.AddValue("fieldWidth", "Ancho del campo (m)", fieldWidth);
    cmd.AddValue("fieldHeight", "Alto del campo (m)", fieldHeight);
    cmd.AddValue("leaderSpeed", "Velocidad del lider de cluster (m/s)", leaderSpeed);
    cmd.AddValue("followerSpeedMax",
                 "Velocidad maxima de seguidores relativa al lider (m/s)",
                 followerSpeedMax);
    cmd.AddValue("clusterBoxWidth", "Ancho de la caja local del cluster (m)", clusterBoxWidth);
    cmd.AddValue("clusterBoxHeight", "Alto de la caja local del cluster (m)", clusterBoxHeight);
    cmd.AddValue("sensorInterval", "Periodo de envio UDP de los sensores (s)", sensorInterval);
    cmd.AddValue("packetSize", "Tamano del paquete UDP de sensores (bytes)", packetSize);
    cmd.AddValue("rngRun", "Indice de corrida RNG para reproducibilidad", rngRun);
    cmd.AddValue("flowmonFile", "Archivo XML de salida para FlowMonitor", flowmonFile);
    cmd.Parse(argc, argv);

    if (agrobotsPerCluster < 1 || sensorsPerField < 1 || simTime <= 5.0 || fieldWidth <= 20.0 ||
        fieldHeight <= 20.0 || clusterBoxWidth <= 1.0 || clusterBoxHeight <= 1.0 ||
        sensorInterval <= 0.0)
    {
        std::cerr << "Parametros invalidos para el experimento." << std::endl;
        return 1;
    }

    RngSeedManager::SetRun(rngRun);

    const uint32_t followersPerCluster = agrobotsPerCluster - 1;
    const uint32_t totalFollowers = nClusters * followersPerCluster;
    const uint32_t totalAgrobots = nClusters * agrobotsPerCluster;
    const Rectangle fieldBounds(0.0, fieldWidth, 0.0, fieldHeight);
    const Rectangle localBounds(-clusterBoxWidth * 0.5,
                                clusterBoxWidth * 0.5,
                                -clusterBoxHeight * 0.5,
                                clusterBoxHeight * 0.5);

    // -------------------------------------------------------------------------
    // NODOS
    // -------------------------------------------------------------------------
    NodeContainer leaders;
    leaders.Create(nClusters);

    NodeContainer followers;
    followers.Create(totalFollowers);

    NodeContainer sensors;
    sensors.Create(sensorsPerField);

    NodeContainer agrobots;
    agrobots.Add(leaders);
    agrobots.Add(followers);

    NodeContainer allNodes;
    allNodes.Add(agrobots);
    allNodes.Add(sensors);

    std::vector<NodeContainer> clusterMembers(nClusters);
    std::vector<NodeContainer> clusterFollowers(nClusters);
    for (uint32_t c = 0; c < nClusters; ++c)
    {
        clusterMembers[c].Add(leaders.Get(c));
        for (uint32_t j = 0; j < followersPerCluster; ++j)
        {
            Ptr<Node> follower = followers.Get(c * followersPerCluster + j);
            clusterFollowers[c].Add(follower);
            clusterMembers[c].Add(follower);
        }
    }

    // -------------------------------------------------------------------------
    // MOVILIDAD
    // -------------------------------------------------------------------------
    // Lideres: trayectoria global sobre el campo.
    // Seguidores: movimiento relativo al lider dentro de una caja local.
    // Sensores: nodos estaticos distribuidos uniformemente en el campo.
    // -------------------------------------------------------------------------
    MobilityHelper leaderMobility;
    leaderMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    leaderMobility.Install(leaders);
    ConfigureLeaderWaypoints(leaders, simTime, fieldBounds, leaderSpeed);

    for (uint32_t c = 0; c < nClusters; ++c)
    {
        InstallFollowerClusterMobility(clusterFollowers[c],
                                       leaders.Get(c)->GetObject<MobilityModel>(),
                                       localBounds,
                                       followerSpeedMax);
    }

    InstallSensorMobility(sensors, fieldBounds);

    // -------------------------------------------------------------------------
    // WiFi ad-hoc + AODV
    // -------------------------------------------------------------------------
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate12Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps"));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default();
    Ptr<YansWifiChannel> wifiChannel = channelHelper.Create();

    YansWifiPhyHelper agrobotPhy;
    agrobotPhy.SetChannel(wifiChannel);
    agrobotPhy.Set("TxPowerStart", DoubleValue(16.0));
    agrobotPhy.Set("TxPowerEnd", DoubleValue(16.0));

    YansWifiPhyHelper sensorPhy;
    sensorPhy.SetChannel(wifiChannel);
    sensorPhy.Set("TxPowerStart", DoubleValue(12.0));
    sensorPhy.Set("TxPowerEnd", DoubleValue(12.0));

    NetDeviceContainer agrobotDevices = wifi.Install(agrobotPhy, mac, agrobots);
    NetDeviceContainer sensorDevices = wifi.Install(sensorPhy, mac, sensors);

    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");

    NetDeviceContainer allDevices;
    allDevices.Add(agrobotDevices);
    allDevices.Add(sensorDevices);
    Ipv4InterfaceContainer ifaces = ipv4.Assign(allDevices);

    // -------------------------------------------------------------------------
    // TRAFICO UDP (sensores -> lideres)
    // -------------------------------------------------------------------------
    const uint16_t sinkPort = 9000;
    ApplicationContainer sinkApps;
    std::vector<Ipv4Address> leaderIps;
    for (uint32_t i = 0; i < nClusters; ++i)
    {
        leaderIps.push_back(ifaces.GetAddress(i));

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), sinkPort));
        sinkApps.Add(sinkHelper.Install(leaders.Get(i)));
    }
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(simTime));

    std::vector<uint32_t> sensorAssignedLeader(sensorsPerField);
    std::vector<uint32_t> sensorsPerLeader(nClusters, 0);
    std::map<uint32_t, uint32_t> sensorIndexByIp;

    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        const Vector sensorPosition = sensors.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const uint32_t leaderIndex = FindNearestLeader(sensorPosition, leaders);
        sensorAssignedLeader[i] = leaderIndex;
        sensorsPerLeader[leaderIndex]++;
        sensorIndexByIp.emplace(ifaces.GetAddress(totalAgrobots + i).Get(), i);
    }

    ApplicationContainer sensorApps;
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        UdpClientHelper client(leaderIps[sensorAssignedLeader[i]], sinkPort);
        client.SetAttribute("MaxPackets",
                            UintegerValue(std::numeric_limits<uint32_t>::max()));
        client.SetAttribute("Interval", TimeValue(Seconds(sensorInterval)));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));

        ApplicationContainer app = client.Install(sensors.Get(i));
        const double start = 2.0 + 0.05 * static_cast<double>(i % 20);
        app.Start(Seconds(start));
        app.Stop(Seconds(simTime - 1.0));
        sensorApps.Add(app);
    }

    // -------------------------------------------------------------------------
    // FLOWMONITOR
    // -------------------------------------------------------------------------
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    uint64_t dataTxPackets = 0;
    uint64_t dataRxPackets = 0;
    uint64_t dataRxDelaySamples = 0;
    double totalDelaySeconds = 0.0;
    std::vector<bool> sensorCovered(sensorsPerField, false);

    for (const auto& [flowId, st] : stats)
    {
        const Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        const bool isSensorDataFlow = (t.protocol == 17) && (t.destinationPort == sinkPort);
        if (!isSensorDataFlow)
        {
            continue;
        }

        dataTxPackets += st.txPackets;
        dataRxPackets += st.rxPackets;
        totalDelaySeconds += st.delaySum.GetSeconds();
        dataRxDelaySamples += st.rxPackets;

        auto it = sensorIndexByIp.find(t.sourceAddress.Get());
        if (it != sensorIndexByIp.end() && st.rxPackets > 0)
        {
            sensorCovered[it->second] = true;
        }
    }

    uint32_t coveredSensors = 0;
    std::vector<uint32_t> coveredSensorsPerLeader(nClusters, 0);
    for (uint32_t i = 0; i < sensorCovered.size(); ++i)
    {
        if (!sensorCovered[i])
        {
            continue;
        }

        coveredSensors++;
        coveredSensorsPerLeader[sensorAssignedLeader[i]]++;
    }

    const double coverage = 100.0 * static_cast<double>(coveredSensors) /
                            static_cast<double>(sensorsPerField);
    const double pdr = (dataTxPackets > 0)
                           ? (100.0 * static_cast<double>(dataRxPackets) /
                              static_cast<double>(dataTxPackets))
                           : 0.0;
    const double avgE2eDelay = (dataRxDelaySamples > 0)
                                   ? (totalDelaySeconds / static_cast<double>(dataRxDelaySamples))
                                   : 0.0;

    std::cout << "\n===== RESULTADOS PRIMER NIVEL MANET JERARQUICA =====\n";
    std::cout << "Clusters: " << nClusters << "\n";
    std::cout << "Agrobots por cluster: " << agrobotsPerCluster << "\n";
    std::cout << "Total agrobots: " << totalAgrobots << "\n";
    std::cout << "Sensores estaticos: " << sensorsPerField << "\n";
    std::cout << "Campo (m): " << fieldWidth << " x " << fieldHeight << "\n";
    std::cout << "Sensores cubiertos: " << coveredSensors << "/" << sensorsPerField << "\n";
    std::cout << "Cobertura (%): " << coverage << "\n";
    std::cout << "Paquetes datos TX: " << dataTxPackets << "\n";
    std::cout << "Paquetes datos RX: " << dataRxPackets << "\n";
    std::cout << "PDR (%): " << pdr << "\n";
    std::cout << "Retardo E2E promedio (s): " << avgE2eDelay << "\n";
    for (uint32_t i = 0; i < nClusters; ++i)
    {
        std::cout << "Cluster " << i << " -> sensores asignados: " << sensorsPerLeader[i]
                  << ", sensores cubiertos: " << coveredSensorsPerLeader[i] << "\n";
    }
    std::cout << "FlowMonitor XML: " << flowmonFile << "\n";

    monitor->SerializeToXmlFile(flowmonFile, true, true);

    Simulator::Destroy();
    return 0;
}
