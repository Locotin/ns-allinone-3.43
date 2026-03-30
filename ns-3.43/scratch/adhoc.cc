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

static void
InstallAgribotClusterMobility(const NodeContainer& cluster,
                              const Rectangle& bounds,
                              double agribotSpeedMax)
{
    // --- Movilidad: Agribots (Nivel 1) ---------------------------------------
    // Se instala RandomWaypointMobilityModel dentro de un rectangulo especifico
    // (bounding box) para representar una zona de cultivo independiente.
    // Cada cluster usa su propio asignador de posiciones aleatorias (X,Y).

    const double maxSpeed = std::max(0.6, agribotSpeedMax);

    Ptr<RandomRectanglePositionAllocator> posAlloc = CreateObject<RandomRectanglePositionAllocator>();
    posAlloc->SetAttribute("X", StringValue(MakeUniformRv(bounds.xMin, bounds.xMax)));
    posAlloc->SetAttribute("Y", StringValue(MakeUniformRv(bounds.yMin, bounds.yMax)));

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed",
                              StringValue(MakeUniformRv(0.5, maxSpeed)),
                              "Pause",
                              StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "PositionAllocator",
                              PointerValue(posAlloc));
    mobility.Install(cluster);
}

static void
ConfigureTractorWaypoints(const NodeContainer& tractors,
                          double simTime,
                          double xMin,
                          double xMax,
                          double y0,
                          double laneSeparation,
                          double legDuration)
{
    // --- Movilidad: Tractores/Superhosts (Nivel 2) ---------------------------
    // Se usa WaypointMobilityModel para imponer trayectorias lineales paralelas.
    // Esto conserva la simetria espacial entre tractores, requisito central de
    // la hipotesis experimental.

    for (uint32_t i = 0; i < tractors.GetN(); ++i)
    {
        Ptr<WaypointMobilityModel> wp = tractors.Get(i)->GetObject<WaypointMobilityModel>();
        if (!wp)
        {
            NS_FATAL_ERROR("WaypointMobilityModel no instalado en tractor " << i);
        }

        const double laneY = y0 + laneSeparation * static_cast<double>(i);

        // Waypoint inicial unico en t=0.
        wp->AddWaypoint(Waypoint(Seconds(0.0), Vector(xMin, laneY, 0.0)));

        // El siguiente waypoint debe tener tiempo estrictamente mayor a 0.
        double t = legDuration;
        double x = xMax;     // primer tramo: xMin -> xMax
        bool toRight = false; // despues alterna hacia xMin, luego xMax, etc.

        // Barrido de ida y vuelta lineal, paralelo para todos los tractores.
        while (t <= simTime + legDuration)
        {
            wp->AddWaypoint(Waypoint(Seconds(t), Vector(x, laneY, 0.0)));
            x = toRight ? xMax : xMin;
            toRight = !toRight;
            t += legDuration;
        }
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    // -------------------------------------------------------------------------
    // Parametros de experimento (CommandLine)
    // -------------------------------------------------------------------------
    // simTime: tiempo total de simulacion (default 300 s).
    // agribotSpeed: velocidad maxima para RandomWaypoint de Agribots.
    // flowmonFile: salida XML de FlowMonitor.
    //
    // Ejemplo de ejecucion:
    // ./ns3 run "scratch/adhoc --simTime=300 --agribotSpeed=4.0"
    // -------------------------------------------------------------------------
    double simTime = 300.0;
    double agribotSpeed = 4.0;
    uint32_t packetSize = 256;
    std::string appDataRate = "40kbps";
    std::string flowmonFile = "flowmon-adhoc.xml";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Tiempo de simulacion en segundos (default: 300)", simTime);
    cmd.AddValue("agribotSpeed",
                 "Velocidad maxima de Agribots (m/s) para RandomWaypoint",
                 agribotSpeed);
    cmd.AddValue("packetSize", "Tamano de paquete UDP (bytes)", packetSize);
    cmd.AddValue("appDataRate", "Tasa de datos de OnOffApplication (ej. 40kbps)", appDataRate);
    cmd.AddValue("flowmonFile", "Archivo XML de salida para FlowMonitor", flowmonFile);
    cmd.Parse(argc, argv);

    // -------------------------------------------------------------------------
    // NODOS
    // -------------------------------------------------------------------------
    // Nivel 1 (Agribots): 15 nodos, organizados en 3 clusters logicos de 5 nodos.
    // Nivel 2 (Tractores/Superhosts): 2 nodos moviles de mayor potencia.
    // -------------------------------------------------------------------------
    const uint32_t nAgribots = 15;
    const uint32_t nClusters = 3;
    const uint32_t agribotsPerCluster = 5;
    const uint32_t nTractors = 2;

    NodeContainer agribots;
    agribots.Create(nAgribots);

    NodeContainer tractors;
    tractors.Create(nTractors);

    std::vector<NodeContainer> agribotClusters(nClusters);
    for (uint32_t c = 0; c < nClusters; ++c)
    {
        for (uint32_t j = 0; j < agribotsPerCluster; ++j)
        {
            agribotClusters[c].Add(agribots.Get(c * agribotsPerCluster + j));
        }
    }

    NodeContainer allNodes;
    allNodes.Add(agribots);
    allNodes.Add(tractors);

    // -------------------------------------------------------------------------
    // MOVILIDAD
    // -------------------------------------------------------------------------
    // 1) Agribots con RandomWaypoint dentro de 3 zonas rectangulares.
    // 2) Tractores con WaypointMobilityModel en barridos lineales paralelos.
    // -------------------------------------------------------------------------
    const Rectangle zoneA(0.0, 80.0, 0.0, 80.0);
    const Rectangle zoneB(120.0, 200.0, 0.0, 80.0);
    const Rectangle zoneC(60.0, 140.0, 100.0, 180.0);

    InstallAgribotClusterMobility(agribotClusters[0], zoneA, agribotSpeed);
    InstallAgribotClusterMobility(agribotClusters[1], zoneB, agribotSpeed);
    InstallAgribotClusterMobility(agribotClusters[2], zoneC, agribotSpeed);

    MobilityHelper tractorMobility;
    tractorMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    tractorMobility.Install(tractors);

    const double xMin = 0.0;
    const double xMax = 200.0;
    const double yLane0 = 40.0;
    const double laneSeparation = 90.0;
    const double legDuration = 25.0;
    ConfigureTractorWaypoints(tractors,
                              simTime,
                              xMin,
                              xMax,
                              yLane0,
                              laneSeparation,
                              legDuration);

    // -------------------------------------------------------------------------
    // WiFi (802.11g ad-hoc) + AODV
    // -------------------------------------------------------------------------
    // Se crea un unico canal ad-hoc y se instalan dispositivos con dos perfiles
    // PHY: Agribots (potencia base) y Tractores (mayor potencia).
    // Luego se instala pila IP con AODV en todos los nodos.
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

    YansWifiPhyHelper agribotPhy;
    agribotPhy.SetChannel(wifiChannel);
    agribotPhy.Set("TxPowerStart", DoubleValue(16.0));
    agribotPhy.Set("TxPowerEnd", DoubleValue(16.0));

    YansWifiPhyHelper tractorPhy;
    tractorPhy.SetChannel(wifiChannel);
    tractorPhy.Set("TxPowerStart", DoubleValue(23.0));
    tractorPhy.Set("TxPowerEnd", DoubleValue(23.0));

    NetDeviceContainer agribotDevices = wifi.Install(agribotPhy, mac, agribots);
    NetDeviceContainer tractorDevices = wifi.Install(tractorPhy, mac, tractors);

    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");

    NetDeviceContainer allDevices;
    allDevices.Add(agribotDevices);
    allDevices.Add(tractorDevices);
    Ipv4InterfaceContainer ifaces = ipv4.Assign(allDevices);

    // -------------------------------------------------------------------------
    // TRAFICO UDP (OnOff -> PacketSink)
    // -------------------------------------------------------------------------
    // Los 15 agribots generan rafagas UDP y envian a los 2 tractores (round-robin).
    // Esto modela telemetria periodica bajo conectividad ad-hoc multi-salto.
    // -------------------------------------------------------------------------
    const uint16_t sinkPort = 9000;
    ApplicationContainer sinkApps;

    std::vector<Ipv4Address> tractorIps;
    for (uint32_t i = 0; i < nTractors; ++i)
    {
        const uint32_t ifaceIndex = nAgribots + i;
        Ipv4Address tractorIp = ifaces.GetAddress(ifaceIndex);
        tractorIps.push_back(tractorIp);

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), sinkPort));
        sinkApps.Add(sinkHelper.Install(tractors.Get(i)));
    }
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(simTime));

    ApplicationContainer onOffApps;
    for (uint32_t i = 0; i < nAgribots; ++i)
    {
        const Ipv4Address dst = tractorIps[i % nTractors];

        OnOffHelper onOff("ns3::UdpSocketFactory", Address());
        onOff.SetAttribute("Remote", AddressValue(InetSocketAddress(dst, sinkPort)));
        onOff.SetAttribute("PacketSize", UintegerValue(packetSize));
        onOff.SetAttribute("DataRate", DataRateValue(DataRate(appDataRate)));
        onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"));

        ApplicationContainer app = onOff.Install(agribots.Get(i));
        const double start = 2.0 + 0.2 * static_cast<double>(i);
        app.Start(Seconds(start));
        app.Stop(Seconds(simTime - 1.0));
        onOffApps.Add(app);
    }

    // -------------------------------------------------------------------------
    // FLOWMONITOR
    // -------------------------------------------------------------------------
    // FlowMonitor captura flujos IPv4 para medir:
    // - PDR de flujos de datos UDP (destino sinkPort)
    // - Retardo extremo a extremo promedio
    // - Sobrecarga de enrutamiento aproximada de AODV (puerto UDP 654)
    // Ademas, exporta un XML para post-procesamiento offline.
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
    uint64_t routingTxPackets = 0;
    double totalDelaySeconds = 0.0;

    for (const auto& [flowId, st] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        // Trafico de control AODV: UDP puerto 654.
        const bool isAodvControl = (t.protocol == 17) && (t.sourcePort == 654 || t.destinationPort == 654);
        if (isAodvControl)
        {
            routingTxPackets += st.txPackets;
            continue;
        }

        // Flujos de datos de la aplicacion: UDP cuyo destino es el sinkPort.
        const bool isDataFlow = (t.protocol == 17) && (t.destinationPort == sinkPort);
        if (isDataFlow)
        {
            dataTxPackets += st.txPackets;
            dataRxPackets += st.rxPackets;
            totalDelaySeconds += st.delaySum.GetSeconds();
            dataRxDelaySamples += st.rxPackets;
        }
    }

    const double pdr = (dataTxPackets > 0) ? (100.0 * static_cast<double>(dataRxPackets) /
                                              static_cast<double>(dataTxPackets))
                                           : 0.0;

    const double avgE2eDelay = (dataRxDelaySamples > 0)
                                   ? (totalDelaySeconds / static_cast<double>(dataRxDelaySamples))
                                   : 0.0;

    // Sobrecarga de enrutamiento: paquetes de control AODV por paquete de datos enviado.
    const double routingOverhead = (dataTxPackets > 0)
                                       ? (100.0 * static_cast<double>(routingTxPackets) /
                                          static_cast<double>(dataTxPackets))
                                       : 0.0;

    std::cout << "\n===== RESULTADOS EXPERIMENTO MANET JERARQUICA =====\n";
    std::cout << "Tiempo simulacion (s): " << simTime << "\n";
    std::cout << "Velocidad maxima Agribot (m/s): " << agribotSpeed << "\n";
    std::cout << "Paquetes datos TX: " << dataTxPackets << "\n";
    std::cout << "Paquetes datos RX: " << dataRxPackets << "\n";
    std::cout << "PDR (%): " << pdr << "\n";
    std::cout << "Retardo E2E promedio (s): " << avgE2eDelay << "\n";
    std::cout << "Overhead AODV (% sobre datos TX): " << routingOverhead << "\n";
    std::cout << "FlowMonitor XML: " << flowmonFile << "\n";

    monitor->SerializeToXmlFile(flowmonFile, true, true);

    Simulator::Destroy();
    return 0;
}