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

enum class RelayMode
{
    BASELINE,
    SCF
};

struct StageMetrics
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t rxDelaySamples{0};
    double totalDelaySeconds{0.0};
};

struct BufferedSample
{
    Ptr<Packet> packet;
    uint64_t key{0};
    Time originTime{Seconds(0)};
    Time enqueueTime{Seconds(0)};
    uint32_t sensorId{0};
    uint32_t sampleId{0};
};

struct LayerAggregateStats
{
    uint32_t nodeCount{0};
    uint64_t generated{0};
    uint64_t received{0};
    uint64_t duplicates{0};
    uint64_t forwarded{0};
    uint64_t overflowDrops{0};
    uint64_t ttlDrops{0};
    uint64_t contactOpportunities{0};
    uint64_t carrySamples{0};
    double carryTimeSumSeconds{0.0};
    double averageOccupancySum{0.0};
    uint32_t maxOccupancy{0};
};

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
    start.WriteHtonU32(m_sensorId);
    start.WriteHtonU32(m_sampleId);
    start.WriteHtonU64(m_originTimestampNs);
    start.WriteU8(m_logicalHop);
}

uint32_t
SampleHeader::Deserialize(Buffer::Iterator start)
{
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

    uint64_t GetGeneratedSamples() const;
    uint64_t GetReceivedSamples() const;
    uint64_t GetDuplicateSamples() const;
    uint64_t GetForwardedSamples() const;
    uint64_t GetOverflowDrops() const;
    uint64_t GetTtlDrops() const;
    uint64_t GetContactOpportunities() const;
    uint64_t GetCarrySamples() const;
    double GetCarryTimeSumSeconds() const;
    double GetAverageOccupancy() const;
    uint32_t GetMaxOccupancy() const;
    uint32_t GetCurrentBufferOccupancy() const;
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
    uint32_t m_maxOccupancy{0};
    uint64_t m_generatedSamples{0};
    uint64_t m_receivedSamples{0};
    uint64_t m_duplicateSamples{0};
    uint64_t m_forwardedSamples{0};
    uint64_t m_overflowDrops{0};
    uint64_t m_ttlDrops{0};
    uint64_t m_contactOpportunities{0};
    uint64_t m_carrySamples{0};
    Time m_carryTimeSum{Seconds(0.0)};
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
    m_isSource = true;
    m_sourceId = sourceId;
    m_generationInterval = generationInterval;
    m_packetSize = packetSize;
    m_initialGenerationDelay = initialGenerationDelay;
}

uint64_t
StoreCarryForwardApp::GetGeneratedSamples() const
{
    return m_generatedSamples;
}

uint64_t
StoreCarryForwardApp::GetReceivedSamples() const
{
    return m_receivedSamples;
}

uint64_t
StoreCarryForwardApp::GetDuplicateSamples() const
{
    return m_duplicateSamples;
}

uint64_t
StoreCarryForwardApp::GetForwardedSamples() const
{
    return m_forwardedSamples;
}

uint64_t
StoreCarryForwardApp::GetOverflowDrops() const
{
    return m_overflowDrops;
}

uint64_t
StoreCarryForwardApp::GetTtlDrops() const
{
    return m_ttlDrops;
}

uint64_t
StoreCarryForwardApp::GetContactOpportunities() const
{
    return m_contactOpportunities;
}

uint64_t
StoreCarryForwardApp::GetCarrySamples() const
{
    return m_carrySamples;
}

double
StoreCarryForwardApp::GetCarryTimeSumSeconds() const
{
    return m_carryTimeSum.GetSeconds();
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
StoreCarryForwardApp::GetMaxOccupancy() const
{
    return m_maxOccupancy;
}

uint32_t
StoreCarryForwardApp::GetCurrentBufferOccupancy() const
{
    return m_buffer.size();
}

uint32_t
StoreCarryForwardApp::GetUniqueSourceCount() const
{
    return m_uniqueSourceIds.size();
}

void
StoreCarryForwardApp::DoDispose()
{
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
    m_running = true;
    m_activeStart = Simulator::Now();
    m_lastOccupancyUpdate = Simulator::Now();

    if (m_listenPort != 0 && !m_rxSocket)
    {
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
    if (!m_running)
    {
        return;
    }

    PurgeExpiredSamples();

    SampleHeader header;
    header.SetSensorId(m_sourceId);
    header.SetSampleId(m_nextSampleId++);
    header.SetOriginTimestamp(Simulator::Now());
    header.SetLogicalHop(m_logicalHop);

    const uint32_t payloadSize =
        (m_packetSize > SampleHeader::SERIALIZED_SIZE) ? (m_packetSize - SampleHeader::SERIALIZED_SIZE)
                                                       : 0;
    Ptr<Packet> packet = Create<Packet>(payloadSize);
    packet->AddHeader(header);

    ++m_generatedSamples;
    EnqueueSample(packet, header, false);

    m_generationEvent =
        Simulator::Schedule(m_generationInterval, &StoreCarryForwardApp::GenerateSample, this);
}

void
StoreCarryForwardApp::CheckContactAndForward()
{
    if (!m_running)
    {
        return;
    }

    PurgeExpiredSamples();

    if (!m_buffer.empty() && m_nextHopNode && m_txSocket && IsInContact())
    {
        ++m_contactOpportunities;

        uint32_t forwardedThisRound = 0;
        while (forwardedThisRound < m_forwardBurstPackets && !m_buffer.empty())
        {
            const BufferedSample record = m_buffer.front();
            SampleHeader header;
            Ptr<Packet> inspect = record.packet->Copy();
            inspect->RemoveHeader(header);
            header.SetLogicalHop(m_logicalHop);
            const uint32_t payloadSize =
                (record.packet->GetSize() > SampleHeader::SERIALIZED_SIZE)
                    ? (record.packet->GetSize() - SampleHeader::SERIALIZED_SIZE)
                    : 0;
            Ptr<Packet> outbound = Create<Packet>(payloadSize);
            outbound->AddHeader(header);

            if (m_txSocket->Send(outbound) < 0)
            {
                break;
            }

            UpdateOccupancyIntegral();
            m_buffer.pop_front();

            ++m_forwardedSamples;
            ++m_carrySamples;
            m_carryTimeSum += (Simulator::Now() - record.enqueueTime);
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
    const uint64_t key = MakeSampleKey(header.GetSensorId(), header.GetSampleId());

    if (countAsReceived)
    {
        if (m_seenSampleKeys.find(key) != m_seenSampleKeys.end())
        {
            ++m_duplicateSamples;
            return false;
        }
        m_seenSampleKeys.insert(key);
        m_uniqueSourceIds.insert(header.GetSensorId());
    }

    if (m_buffer.size() >= m_bufferCapacityPackets)
    {
        ++m_overflowDrops;
        return false;
    }

    UpdateOccupancyIntegral();

    BufferedSample sample;
    sample.packet = packet->Copy();
    sample.key = key;
    sample.originTime = header.GetOriginTimestamp();
    sample.enqueueTime = Simulator::Now();
    sample.sensorId = header.GetSensorId();
    sample.sampleId = header.GetSampleId();
    m_buffer.push_back(sample);
    m_maxOccupancy = std::max<uint32_t>(m_maxOccupancy, m_buffer.size());

    if (countAsReceived)
    {
        ++m_receivedSamples;
    }

    return true;
}

void
StoreCarryForwardApp::PurgeExpiredSamples()
{
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

    uint64_t GetDeliveredSamples() const;
    uint64_t GetDuplicateSamples() const;
    uint64_t GetStaleSamples() const;
    uint32_t GetSensorsDeliveredToHouse() const;
    double GetAverageEndToEndDelay() const;
    double GetMinEndToEndDelay() const;
    double GetMaxEndToEndDelay() const;
    double GetAverageAoi() const;
    double GetPeakAoi() const;
    double GetCurrentAverageAoi() const;
    double GetCurrentPeakAoi() const;

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
    uint64_t m_deliveredSamples{0};
    uint64_t m_duplicateSamples{0};
    uint64_t m_staleSamples{0};
    double m_delaySumSeconds{0.0};
    uint64_t m_delaySamples{0};
    double m_minDelaySeconds{std::numeric_limits<double>::max()};
    double m_maxDelaySeconds{0.0};
    double m_aoiSnapshotSumSeconds{0.0};
    uint64_t m_aoiSnapshotSamples{0};
    double m_peakAoiSeconds{0.0};
};

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
    m_listenPort = listenPort;
    m_sensorCount = sensorCount;
    m_aoiSamplePeriod = aoiSamplePeriod;
    m_sensorDelivered.assign(sensorCount, false);
    m_hasFreshUpdate.assign(sensorCount, false);
    m_latestOriginTimestamp.assign(sensorCount, Seconds(0.0));
}

uint64_t
HouseCollectorApp::GetDeliveredSamples() const
{
    return m_deliveredSamples;
}

uint64_t
HouseCollectorApp::GetDuplicateSamples() const
{
    return m_duplicateSamples;
}

uint64_t
HouseCollectorApp::GetStaleSamples() const
{
    return m_staleSamples;
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
HouseCollectorApp::GetMinEndToEndDelay() const
{
    if (m_delaySamples == 0)
    {
        return 0.0;
    }
    return m_minDelaySeconds;
}

double
HouseCollectorApp::GetMaxEndToEndDelay() const
{
    return m_maxDelaySeconds;
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
HouseCollectorApp::GetPeakAoi() const
{
    return m_peakAoiSeconds;
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

double
HouseCollectorApp::GetCurrentPeakAoi() const
{
    double peakAoi = 0.0;

    for (uint32_t i = 0; i < m_sensorCount; ++i)
    {
        if (!m_hasFreshUpdate[i])
        {
            continue;
        }
        peakAoi = std::max(peakAoi,
                           (Simulator::Now() - m_latestOriginTimestamp[i]).GetSeconds());
    }

    return peakAoi;
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
            ++m_duplicateSamples;
            continue;
        }
        m_deliveredKeys.insert(key);
        ++m_deliveredSamples;

        const double delaySeconds = (Simulator::Now() - header.GetOriginTimestamp()).GetSeconds();
        m_delaySumSeconds += delaySeconds;
        ++m_delaySamples;
        m_minDelaySeconds = std::min(m_minDelaySeconds, delaySeconds);
        m_maxDelaySeconds = std::max(m_maxDelaySeconds, delaySeconds);

        if (header.GetSensorId() < m_sensorCount)
        {
            m_sensorDelivered[header.GetSensorId()] = true;
            if (!m_hasFreshUpdate[header.GetSensorId()] ||
                header.GetOriginTimestamp() > m_latestOriginTimestamp[header.GetSensorId()])
            {
                m_hasFreshUpdate[header.GetSensorId()] = true;
                m_latestOriginTimestamp[header.GetSensorId()] = header.GetOriginTimestamp();
            }
            else
            {
                ++m_staleSamples;
            }
        }
    }

    RecordAoiSnapshot();
}

void
HouseCollectorApp::SampleAoi()
{
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
    double sumAoi = 0.0;
    double maxAoi = 0.0;
    uint32_t activeSensors = 0;

    for (uint32_t i = 0; i < m_sensorCount; ++i)
    {
        if (!m_hasFreshUpdate[i])
        {
            continue;
        }

        const double ageSeconds = (Simulator::Now() - m_latestOriginTimestamp[i]).GetSeconds();
        sumAoi += ageSeconds;
        maxAoi = std::max(maxAoi, ageSeconds);
        ++activeSensors;
    }

    if (activeSensors > 0)
    {
        m_aoiSnapshotSumSeconds += sumAoi / static_cast<double>(activeSensors);
        ++m_aoiSnapshotSamples;
        m_peakAoiSeconds = std::max(m_peakAoiSeconds, maxAoi);
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
ConfigureSweepWaypoints(const NodeContainer& leaders,
                        double simTime,
                        const Rectangle& fieldBounds,
                        double leaderSpeed,
                        bool horizontalSweep)
{
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
InstallRelativeClusterMobility(const NodeContainer& followers,
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

static void
InstallFixedMobility(const NodeContainer& nodes, const std::vector<Vector>& positions)
{
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
AssignLevel1LeadersToLevel2Clusters(const NodeContainer& level1Leaders, const NodeContainer& level2Leaders)
{
    const uint32_t invalidAssignment = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> assignment(level1Leaders.GetN(), invalidAssignment);
    std::vector<bool> level1Taken(level1Leaders.GetN(), false);

    for (uint32_t level2Index = 0; level2Index < level2Leaders.GetN(); ++level2Index)
    {
        const Vector level2Position = level2Leaders.Get(level2Index)->GetObject<MobilityModel>()->GetPosition();
        uint32_t bestLevel1Index = invalidAssignment;
        double bestDistance2 = std::numeric_limits<double>::max();

        for (uint32_t level1Index = 0; level1Index < level1Leaders.GetN(); ++level1Index)
        {
            if (level1Taken[level1Index])
            {
                continue;
            }

            const Vector level1Position =
                level1Leaders.Get(level1Index)->GetObject<MobilityModel>()->GetPosition();
            const double dx = level1Position.x - level2Position.x;
            const double dy = level1Position.y - level2Position.y;
            const double distance2 = dx * dx + dy * dy;

            if (distance2 < bestDistance2)
            {
                bestDistance2 = distance2;
                bestLevel1Index = level1Index;
            }
        }

        if (bestLevel1Index != invalidAssignment)
        {
            assignment[bestLevel1Index] = level2Index;
            level1Taken[bestLevel1Index] = true;
        }
    }

    for (uint32_t level1Index = 0; level1Index < level1Leaders.GetN(); ++level1Index)
    {
        if (assignment[level1Index] != invalidAssignment)
        {
            continue;
        }

        const Vector level1Position = level1Leaders.Get(level1Index)->GetObject<MobilityModel>()->GetPosition();
        assignment[level1Index] = FindNearestNode(level1Position, level2Leaders);
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

static uint32_t
CountDeliveredSources(const std::vector<bool>& delivered)
{
    return std::count(delivered.begin(), delivered.end(), true);
}

static double
ComputePdr(const StageMetrics& metrics)
{
    if (metrics.txPackets == 0)
    {
        return 0.0;
    }

    return 100.0 * static_cast<double>(metrics.rxPackets) / static_cast<double>(metrics.txPackets);
}

static double
ComputeAverageDelay(const StageMetrics& metrics)
{
    if (metrics.rxDelaySamples == 0)
    {
        return 0.0;
    }

    return metrics.totalDelaySeconds / static_cast<double>(metrics.rxDelaySamples);
}

static LayerAggregateStats
AggregateLayerStats(const std::vector<Ptr<StoreCarryForwardApp>>& apps)
{
    LayerAggregateStats stats;
    for (const auto& app : apps)
    {
        stats.nodeCount++;
        stats.generated += app->GetGeneratedSamples();
        stats.received += app->GetReceivedSamples();
        stats.duplicates += app->GetDuplicateSamples();
        stats.forwarded += app->GetForwardedSamples();
        stats.overflowDrops += app->GetOverflowDrops();
        stats.ttlDrops += app->GetTtlDrops();
        stats.contactOpportunities += app->GetContactOpportunities();
        stats.carrySamples += app->GetCarrySamples();
        stats.carryTimeSumSeconds += app->GetCarryTimeSumSeconds();
        stats.averageOccupancySum += app->GetAverageOccupancy();
        stats.maxOccupancy = std::max(stats.maxOccupancy, app->GetMaxOccupancy());
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

static double
ComputeAverageCarryTime(const LayerAggregateStats& stats)
{
    if (stats.carrySamples == 0)
    {
        return 0.0;
    }
    return stats.carryTimeSumSeconds / static_cast<double>(stats.carrySamples);
}

static bool
FileHasContent(const std::string& fileName)
{
    std::ifstream input(fileName);
    return input.good() && input.peek() != std::ifstream::traits_type::eof();
}

static void
EnsureCsvHeader(const std::string& fileName, const std::string& headerLine)
{
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
    std::ofstream output(fileName, std::ios::app);
    output << line << "\n";
}

static std::string
FormatDouble(double value, uint32_t precision = 6)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

static std::string
JoinCsvFields(const std::vector<std::string>& fields)
{
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
BuildDefaultRunLabel(const std::string& relayMode, uint32_t rngRun)
{
    std::ostringstream oss;
    oss << relayMode << "-run" << rngRun;
    return oss.str();
}

static uint32_t
CountCoveredSensorsAtLevel1(const std::vector<Ptr<StoreCarryForwardApp>>& level1Apps)
{
    uint32_t coveredSensors = 0;
    for (const auto& app : level1Apps)
    {
        coveredSensors += app->GetUniqueSourceCount();
    }
    return coveredSensors;
}

static uint64_t
SumCurrentBufferOccupancy(const std::vector<Ptr<StoreCarryForwardApp>>& apps)
{
    uint64_t totalOccupancy = 0;
    for (const auto& app : apps)
    {
        totalOccupancy += app->GetCurrentBufferOccupancy();
    }
    return totalOccupancy;
}

static double
ComputeCurrentAverageBufferOccupancy(const std::vector<Ptr<StoreCarryForwardApp>>& apps)
{
    if (apps.empty())
    {
        return 0.0;
    }

    return static_cast<double>(SumCurrentBufferOccupancy(apps)) / static_cast<double>(apps.size());
}

static RelayMode
ParseRelayMode(const std::string& relayMode)
{
    if (relayMode == "baseline")
    {
        return RelayMode::BASELINE;
    }
    if (relayMode == "scf")
    {
        return RelayMode::SCF;
    }

    NS_FATAL_ERROR("relayMode invalido: " << relayMode << ". Use baseline o scf.");
    return RelayMode::SCF;
}

} // namespace

int
main(int argc, char* argv[])
{
    const uint32_t minUdpClientPacketSize = 12;
    const uint32_t minClusterNodes = 2;
    const uint32_t nLevel1Clusters = 3;
    const uint32_t nLevel2Clusters = 2;

    // -------------------------------------------------------------------------
    // Parametros de experimento
    // -------------------------------------------------------------------------
    // Nivel 1: 3 clusters de agrobots que recolectan datos de sensores.
    // Nivel 2: 2 clusters terrestres que reciben desde lideres del nivel 1.
    // Casa: sumidero fijo al centro de la finca.
    // Toda la conectividad usa una sola MANET Wi-Fi ad hoc con AODV.
    // relayMode=scf cambia metodologicamente el experimento a store-carry-forward.
    // -------------------------------------------------------------------------
    double simTime = 300.0;
    uint32_t agrobotsPerCluster = 5;
    uint32_t level2NodesPerCluster = 3;
    uint32_t sensorsPerField = 24;
    double fieldWidth = 240.0;
    double fieldHeight = 180.0;
    double leaderSpeed = 2.0;
    double followerSpeedMax = 1.2;
    double level2LeaderSpeed = 1.4;
    double level2FollowerSpeedMax = 0.8;
    double clusterBoxWidth = 30.0;
    double clusterBoxHeight = 20.0;
    double level2BoxWidth = 12.0;
    double level2BoxHeight = 8.0;
    double sensorInterval = 3.0;
    double level1ToLevel2Interval = 4.0;
    double level2ToHouseInterval = 5.0;
    uint32_t packetSize = 128;
    uint32_t level1ToLevel2PacketSize = 192;
    uint32_t level2ToHousePacketSize = 256;
    uint32_t rngRun = 1;
    std::string flowmonFile = "flowmon-adhoc.xml";
    std::string relayMode = "scf";
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

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Tiempo total de simulacion en segundos", simTime);
    cmd.AddValue("agrobotsPerCluster",
                 "Cantidad de nodos por cluster del nivel 1 (incluye al lider)",
                 agrobotsPerCluster);
    cmd.AddValue("level2NodesPerCluster",
                 "Cantidad de nodos por cluster del nivel 2 (incluye al lider)",
                 level2NodesPerCluster);
    cmd.AddValue("sensorsPerField", "Cantidad de sensores estaticos en el campo", sensorsPerField);
    cmd.AddValue("fieldWidth", "Ancho del campo (m)", fieldWidth);
    cmd.AddValue("fieldHeight", "Alto del campo (m)", fieldHeight);
    cmd.AddValue("leaderSpeed", "Velocidad del lider del nivel 1 (m/s)", leaderSpeed);
    cmd.AddValue("followerSpeedMax",
                 "Velocidad maxima de seguidores del nivel 1 (m/s)",
                 followerSpeedMax);
    cmd.AddValue("level2LeaderSpeed", "Velocidad del lider del nivel 2 (m/s)", level2LeaderSpeed);
    cmd.AddValue("level2FollowerSpeedMax",
                 "Velocidad maxima de seguidores del nivel 2 (m/s)",
                 level2FollowerSpeedMax);
    cmd.AddValue("clusterBoxWidth", "Ancho de la caja local del cluster del nivel 1 (m)", clusterBoxWidth);
    cmd.AddValue("clusterBoxHeight", "Alto de la caja local del cluster del nivel 1 (m)", clusterBoxHeight);
    cmd.AddValue("level2BoxWidth", "Ancho de la caja local del cluster del nivel 2 (m)", level2BoxWidth);
    cmd.AddValue("level2BoxHeight", "Alto de la caja local del cluster del nivel 2 (m)", level2BoxHeight);
    cmd.AddValue("sensorInterval", "Periodo de muestreo/generacion por sensor (s)", sensorInterval);
    cmd.AddValue("level1ToLevel2Interval",
                 "Periodo de envio UDP nivel1->nivel2 en modo baseline (s)",
                 level1ToLevel2Interval);
    cmd.AddValue("level2ToHouseInterval",
                 "Periodo de envio UDP nivel2->casa en modo baseline (s)",
                 level2ToHouseInterval);
    cmd.AddValue("packetSize",
                 "Tamano de paquete de muestra sensor->casa en modo scf o sensores->nivel1 en baseline (bytes)",
                 packetSize);
    cmd.AddValue("level1ToLevel2PacketSize",
                 "Tamano de paquete UDP nivel1->nivel2 en modo baseline (bytes)",
                 level1ToLevel2PacketSize);
    cmd.AddValue("level2ToHousePacketSize",
                 "Tamano de paquete UDP nivel2->casa en modo baseline (bytes)",
                 level2ToHousePacketSize);
    cmd.AddValue("rngRun", "Indice de corrida RNG para reproducibilidad", rngRun);
    cmd.AddValue("flowmonFile", "Archivo XML de salida para FlowMonitor", flowmonFile);
    cmd.AddValue("relayMode", "Modo de relevo: baseline o scf", relayMode);
    cmd.AddValue("sensorContactRange",
                 "Umbral de contacto sensor->lider nivel 1 para store-carry-forward (m)",
                 sensorContactRange);
    cmd.AddValue("level1ToLevel2ContactRange",
                 "Umbral de contacto lider nivel 1 -> lider nivel 2 para store-carry-forward (m)",
                 level1ToLevel2ContactRange);
    cmd.AddValue("level2ToHouseContactRange",
                 "Umbral de contacto lider nivel 2 -> casa para store-carry-forward (m)",
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

    const RelayMode selectedRelayMode = ParseRelayMode(relayMode);
    const uint32_t minScfPacketSize = SampleHeader::SERIALIZED_SIZE;
    if (metricsRunLabel.empty())
    {
        metricsRunLabel = BuildDefaultRunLabel(relayMode, rngRun);
    }

    if (agrobotsPerCluster < minClusterNodes || level2NodesPerCluster < minClusterNodes ||
        sensorsPerField < 1 || simTime <= 5.0 || fieldWidth <= 20.0 || fieldHeight <= 20.0 ||
        clusterBoxWidth <= 1.0 || clusterBoxHeight <= 1.0 || level2BoxWidth <= 1.0 ||
        level2BoxHeight <= 1.0 || sensorInterval <= 0.0 || level1ToLevel2Interval <= 0.0 ||
        level2ToHouseInterval <= 0.0 || contactCheckInterval <= 0.0 || forwardBurstPackets < 1 ||
        bufferCapacityPackets < 1 || sampleTtl <= 0.0 || aoiSamplePeriod <= 0.0 ||
        metricsSampleInterval <= 0.0)
    {
        std::cerr << "Parametros invalidos para el experimento." << std::endl;
        return 1;
    }

    if (selectedRelayMode == RelayMode::BASELINE &&
        (packetSize < minUdpClientPacketSize || level1ToLevel2PacketSize < minUdpClientPacketSize ||
         level2ToHousePacketSize < minUdpClientPacketSize))
    {
        std::cerr << "En modo baseline, los tamanos de paquete UDP deben ser >= "
                  << minUdpClientPacketSize << " bytes." << std::endl;
        return 1;
    }

    if (selectedRelayMode == RelayMode::SCF && packetSize < minScfPacketSize)
    {
        std::cerr << "En modo scf, packetSize debe ser >= " << minScfPacketSize
                  << " bytes para alojar SampleHeader." << std::endl;
        return 1;
    }

    RngSeedManager::SetRun(rngRun);

    const uint32_t level1FollowersPerCluster = agrobotsPerCluster - 1;
    const uint32_t level2FollowersPerCluster = level2NodesPerCluster - 1;
    const uint32_t totalLevel1Nodes = nLevel1Clusters * agrobotsPerCluster;
    const uint32_t totalLevel2Nodes = nLevel2Clusters * level2NodesPerCluster;
    const Rectangle fieldBounds(0.0, fieldWidth, 0.0, fieldHeight);
    const Rectangle level1LocalBounds(-clusterBoxWidth * 0.5,
                                      clusterBoxWidth * 0.5,
                                      -clusterBoxHeight * 0.5,
                                      clusterBoxHeight * 0.5);
    const Rectangle level2LocalBounds(-level2BoxWidth * 0.5,
                                      level2BoxWidth * 0.5,
                                      -level2BoxHeight * 0.5,
                                      level2BoxHeight * 0.5);
    const Vector housePosition(fieldWidth * 0.5, fieldHeight * 0.5, 0.0);

    NodeContainer level1Leaders;
    level1Leaders.Create(nLevel1Clusters);

    NodeContainer level1Followers;
    level1Followers.Create(nLevel1Clusters * level1FollowersPerCluster);

    NodeContainer level2Leaders;
    level2Leaders.Create(nLevel2Clusters);

    NodeContainer level2Followers;
    level2Followers.Create(nLevel2Clusters * level2FollowersPerCluster);

    NodeContainer sensors;
    sensors.Create(sensorsPerField);

    NodeContainer house;
    house.Create(1);

    NodeContainer level1Nodes;
    level1Nodes.Add(level1Leaders);
    level1Nodes.Add(level1Followers);

    NodeContainer level2Nodes;
    level2Nodes.Add(level2Leaders);
    level2Nodes.Add(level2Followers);

    NodeContainer allNodes;
    allNodes.Add(level1Nodes);
    allNodes.Add(level2Nodes);
    allNodes.Add(sensors);
    allNodes.Add(house);

    std::vector<NodeContainer> level1ClusterFollowers(nLevel1Clusters);
    std::vector<NodeContainer> level2ClusterFollowers(nLevel2Clusters);
    for (uint32_t c = 0; c < nLevel1Clusters; ++c)
    {
        for (uint32_t j = 0; j < level1FollowersPerCluster; ++j)
        {
            level1ClusterFollowers[c].Add(level1Followers.Get(c * level1FollowersPerCluster + j));
        }
    }

    for (uint32_t c = 0; c < nLevel2Clusters; ++c)
    {
        for (uint32_t j = 0; j < level2FollowersPerCluster; ++j)
        {
            level2ClusterFollowers[c].Add(level2Followers.Get(c * level2FollowersPerCluster + j));
        }
    }

    // -------------------------------------------------------------------------
    // MOVILIDAD
    // -------------------------------------------------------------------------
    // Ambos niveles preservan movilidad de cluster y de nodo.
    // Nivel 1: lideres con barrido horizontal; seguidores con movilidad relativa.
    // Nivel 2: lideres con barrido vertical; seguidores con movilidad relativa.
    // -------------------------------------------------------------------------
    MobilityHelper level1LeaderMobility;
    level1LeaderMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    level1LeaderMobility.Install(level1Leaders);
    ConfigureSweepWaypoints(level1Leaders, simTime, fieldBounds, leaderSpeed, true);

    for (uint32_t c = 0; c < nLevel1Clusters; ++c)
    {
        InstallRelativeClusterMobility(level1ClusterFollowers[c],
                                       level1Leaders.Get(c)->GetObject<MobilityModel>(),
                                       level1LocalBounds,
                                       followerSpeedMax);
    }

    MobilityHelper level2LeaderMobility;
    level2LeaderMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    level2LeaderMobility.Install(level2Leaders);
    ConfigureSweepWaypoints(level2Leaders, simTime, fieldBounds, level2LeaderSpeed, false);

    for (uint32_t c = 0; c < nLevel2Clusters; ++c)
    {
        InstallRelativeClusterMobility(level2ClusterFollowers[c],
                                       level2Leaders.Get(c)->GetObject<MobilityModel>(),
                                       level2LocalBounds,
                                       level2FollowerSpeedMax);
    }

    InstallSensorMobility(sensors, fieldBounds);
    InstallFixedMobility(house, {housePosition});

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

    YansWifiChannelHelper channelHelper;
    channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channelHelper.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                     "ReferenceLoss",
                                     DoubleValue(40.046));
    Ptr<YansWifiChannel> wifiChannel = channelHelper.Create();

    YansWifiPhyHelper level1Phy;
    level1Phy.SetChannel(wifiChannel);
    level1Phy.Set("TxPowerStart", DoubleValue(16.0));
    level1Phy.Set("TxPowerEnd", DoubleValue(16.0));

    YansWifiPhyHelper level2Phy;
    level2Phy.SetChannel(wifiChannel);
    level2Phy.Set("TxPowerStart", DoubleValue(18.0));
    level2Phy.Set("TxPowerEnd", DoubleValue(18.0));

    YansWifiPhyHelper sensorPhy;
    sensorPhy.SetChannel(wifiChannel);
    sensorPhy.Set("TxPowerStart", DoubleValue(12.0));
    sensorPhy.Set("TxPowerEnd", DoubleValue(12.0));

    YansWifiPhyHelper housePhy;
    housePhy.SetChannel(wifiChannel);
    housePhy.Set("TxPowerStart", DoubleValue(20.0));
    housePhy.Set("TxPowerEnd", DoubleValue(20.0));

    NetDeviceContainer level1Devices = wifi.Install(level1Phy, mac, level1Nodes);
    NetDeviceContainer level2Devices = wifi.Install(level2Phy, mac, level2Nodes);
    NetDeviceContainer sensorDevices = wifi.Install(sensorPhy, mac, sensors);
    NetDeviceContainer houseDevices = wifi.Install(housePhy, mac, house);

    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");

    NetDeviceContainer allDevices;
    allDevices.Add(level1Devices);
    allDevices.Add(level2Devices);
    allDevices.Add(sensorDevices);
    allDevices.Add(houseDevices);
    Ipv4InterfaceContainer ifaces = ipv4.Assign(allDevices);

    const uint32_t level2LeaderStartIndex = totalLevel1Nodes;
    const uint32_t sensorStartIndex = totalLevel1Nodes + totalLevel2Nodes;
    const uint32_t houseIndex = sensorStartIndex + sensorsPerField;

    const uint16_t sensorToL1Port = 9000;
    const uint16_t level1ToLevel2Port = 9001;
    const uint16_t level2ToHousePort = 9002;

    std::vector<Ipv4Address> level1LeaderIps;
    std::vector<Ipv4Address> level2LeaderIps;
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        level1LeaderIps.push_back(ifaces.GetAddress(i));
    }
    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        level2LeaderIps.push_back(ifaces.GetAddress(level2LeaderStartIndex + i));
    }
    const Ipv4Address houseIp = ifaces.GetAddress(houseIndex);

    std::vector<uint32_t> sensorAssignedLevel1(sensorsPerField);
    std::vector<uint32_t> sensorsPerLevel1(nLevel1Clusters, 0);
    std::vector<uint32_t> level1AssignedLevel2(nLevel1Clusters);
    std::vector<uint32_t> level1PerLevel2(nLevel2Clusters, 0);

    std::map<uint32_t, uint32_t> sensorIndexByIp;
    std::map<uint32_t, uint32_t> level1LeaderIndexByIp;
    std::map<uint32_t, uint32_t> level2LeaderIndexByIp;

    const std::vector<uint32_t> balancedAssignments =
        AssignLevel1LeadersToLevel2Clusters(level1Leaders, level2Leaders);
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        level1LeaderIndexByIp.emplace(level1LeaderIps[i].Get(), i);
        level1AssignedLevel2[i] = balancedAssignments[i];
        level1PerLevel2[level1AssignedLevel2[i]]++;
    }

    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        level2LeaderIndexByIp.emplace(level2LeaderIps[i].Get(), i);
    }

    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        const Vector sensorPosition = sensors.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const uint32_t level1Index = FindNearestNode(sensorPosition, level1Leaders);
        sensorAssignedLevel1[i] = level1Index;
        sensorsPerLevel1[level1Index]++;
        sensorIndexByIp.emplace(ifaces.GetAddress(sensorStartIndex + i).Get(), i);
    }

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    std::vector<Ptr<StoreCarryForwardApp>> sensorStoreApps;
    std::vector<Ptr<StoreCarryForwardApp>> level1StoreApps;
    std::vector<Ptr<StoreCarryForwardApp>> level2StoreApps;
    Ptr<HouseCollectorApp> houseCollectorApp;
    ApplicationContainer baselineApps;

    if (selectedRelayMode == RelayMode::BASELINE)
    {
        ApplicationContainer stage1Sinks;
        for (uint32_t i = 0; i < nLevel1Clusters; ++i)
        {
            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), sensorToL1Port));
            stage1Sinks.Add(sinkHelper.Install(level1Leaders.Get(i)));
        }

        ApplicationContainer stage2Sinks;
        for (uint32_t i = 0; i < nLevel2Clusters; ++i)
        {
            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), level1ToLevel2Port));
            stage2Sinks.Add(sinkHelper.Install(level2Leaders.Get(i)));
        }

        PacketSinkHelper houseSinkHelper("ns3::UdpSocketFactory",
                                         InetSocketAddress(Ipv4Address::GetAny(), level2ToHousePort));
        ApplicationContainer stage3Sinks = houseSinkHelper.Install(house.Get(0));

        stage1Sinks.Start(Seconds(0.5));
        stage2Sinks.Start(Seconds(0.5));
        stage3Sinks.Start(Seconds(0.5));
        stage1Sinks.Stop(Seconds(simTime));
        stage2Sinks.Stop(Seconds(simTime));
        stage3Sinks.Stop(Seconds(simTime));
        baselineApps.Add(stage1Sinks);
        baselineApps.Add(stage2Sinks);
        baselineApps.Add(stage3Sinks);

        for (uint32_t i = 0; i < sensors.GetN(); ++i)
        {
            UdpClientHelper client(level1LeaderIps[sensorAssignedLevel1[i]], sensorToL1Port);
            client.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
            client.SetAttribute("Interval", TimeValue(Seconds(sensorInterval)));
            client.SetAttribute("PacketSize", UintegerValue(packetSize));

            ApplicationContainer app = client.Install(sensors.Get(i));
            app.Start(Seconds(1.5 + 0.05 * static_cast<double>(i % 20)));
            app.Stop(Seconds(simTime - 1.0));
            baselineApps.Add(app);
        }

        for (uint32_t i = 0; i < nLevel1Clusters; ++i)
        {
            UdpClientHelper client(level2LeaderIps[level1AssignedLevel2[i]], level1ToLevel2Port);
            client.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
            client.SetAttribute("Interval", TimeValue(Seconds(level1ToLevel2Interval)));
            client.SetAttribute("PacketSize", UintegerValue(level1ToLevel2PacketSize));

            ApplicationContainer app = client.Install(level1Leaders.Get(i));
            app.Start(Seconds(2.5 + 0.2 * static_cast<double>(i)));
            app.Stop(Seconds(simTime - 1.0));
            baselineApps.Add(app);
        }

        for (uint32_t i = 0; i < nLevel2Clusters; ++i)
        {
            UdpClientHelper client(houseIp, level2ToHousePort);
            client.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
            client.SetAttribute("Interval", TimeValue(Seconds(level2ToHouseInterval)));
            client.SetAttribute("PacketSize", UintegerValue(level2ToHousePacketSize));

            ApplicationContainer app = client.Install(level2Leaders.Get(i));
            app.Start(Seconds(3.5 + 0.2 * static_cast<double>(i)));
            app.Stop(Seconds(simTime - 1.0));
            baselineApps.Add(app);
        }
    }
    else
    {
        for (uint32_t i = 0; i < sensors.GetN(); ++i)
        {
            Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
            app->SetRoleName("sensor");
            app->SetForwardHop(0);
            app->SetNextHop(level1Leaders.Get(sensorAssignedLevel1[i]),
                            level1LeaderIps[sensorAssignedLevel1[i]],
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
            app->SetStartTime(Seconds(0.5));
            app->SetStopTime(Seconds(simTime - 0.5));
            sensorStoreApps.push_back(app);
        }

        for (uint32_t i = 0; i < nLevel1Clusters; ++i)
        {
            Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
            app->SetRoleName("relay-n1");
            app->SetListenPort(sensorToL1Port);
            app->SetForwardHop(1);
            app->SetNextHop(level2Leaders.Get(level1AssignedLevel2[i]),
                            level2LeaderIps[level1AssignedLevel2[i]],
                            level1ToLevel2Port);
            app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                                Seconds(sampleTtl),
                                                Seconds(contactCheckInterval),
                                                forwardBurstPackets,
                                                level1ToLevel2ContactRange);
            level1Leaders.Get(i)->AddApplication(app);
            app->SetStartTime(Seconds(0.5));
            app->SetStopTime(Seconds(simTime - 0.25));
            level1StoreApps.push_back(app);
        }

        for (uint32_t i = 0; i < nLevel2Clusters; ++i)
        {
            Ptr<StoreCarryForwardApp> app = CreateObject<StoreCarryForwardApp>();
            app->SetRoleName("relay-n2");
            app->SetListenPort(level1ToLevel2Port);
            app->SetForwardHop(2);
            app->SetNextHop(house.Get(0), houseIp, level2ToHousePort);
            app->SetStoreCarryForwardParameters(bufferCapacityPackets,
                                                Seconds(sampleTtl),
                                                Seconds(contactCheckInterval),
                                                forwardBurstPackets,
                                                level2ToHouseContactRange);
            level2Leaders.Get(i)->AddApplication(app);
            app->SetStartTime(Seconds(0.5));
            app->SetStopTime(Seconds(simTime - 0.15));
            level2StoreApps.push_back(app);
        }

        houseCollectorApp = CreateObject<HouseCollectorApp>();
        houseCollectorApp->Configure(level2ToHousePort, sensorsPerField, Seconds(aoiSamplePeriod));
        house.Get(0)->AddApplication(houseCollectorApp);
        houseCollectorApp->SetStartTime(Seconds(0.5));
        houseCollectorApp->SetStopTime(Seconds(simTime));
    }

    std::vector<std::string> timeseriesHeaderFields = {"run_label",
                                                       "relay_mode",
                                                       "time_s",
                                                       "n1_covered_sensors",
                                                       "n1_coverage_pct",
                                                       "house_delivered_samples",
                                                       "house_delivered_sensors",
                                                       "house_coverage_pct",
                                                       "house_e2e_delay_avg_s",
                                                       "house_aoi_avg_s",
                                                       "house_aoi_peak_s",
                                                       "sensor_generated",
                                                       "sensor_forwarded",
                                                       "sensor_contacts",
                                                       "sensor_avg_buffer_occ_now",
                                                       "sensor_overflow_drops",
                                                       "sensor_ttl_drops",
                                                       "n1_received_samples",
                                                       "n1_forwarded_samples",
                                                       "n1_contacts",
                                                       "n1_avg_buffer_occ_now",
                                                       "n1_overflow_drops",
                                                       "n1_ttl_drops",
                                                       "n2_received_samples",
                                                       "n2_forwarded_samples",
                                                       "n2_contacts",
                                                       "n2_avg_buffer_occ_now",
                                                       "n2_overflow_drops",
                                                       "n2_ttl_drops"};

    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        const std::string prefix = "n1_cluster_" + std::to_string(i + 1) + "_";
        timeseriesHeaderFields.push_back(prefix + "assigned_sensors");
        timeseriesHeaderFields.push_back(prefix + "covered_sensors");
        timeseriesHeaderFields.push_back(prefix + "received_samples");
        timeseriesHeaderFields.push_back(prefix + "forwarded_samples");
        timeseriesHeaderFields.push_back(prefix + "buffer_occ_now");
        timeseriesHeaderFields.push_back(prefix + "overflow_drops");
        timeseriesHeaderFields.push_back(prefix + "ttl_drops");
    }

    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        const std::string prefix = "n2_cluster_" + std::to_string(i + 1) + "_";
        timeseriesHeaderFields.push_back(prefix + "assigned_n1_leaders");
        timeseriesHeaderFields.push_back(prefix + "unique_sensor_sources");
        timeseriesHeaderFields.push_back(prefix + "received_samples");
        timeseriesHeaderFields.push_back(prefix + "forwarded_samples");
        timeseriesHeaderFields.push_back(prefix + "buffer_occ_now");
        timeseriesHeaderFields.push_back(prefix + "overflow_drops");
        timeseriesHeaderFields.push_back(prefix + "ttl_drops");
    }

    EnsureCsvHeader(timeseriesCsvFile, JoinCsvFields(timeseriesHeaderFields));

    std::function<void()> snapshotMetrics;
    snapshotMetrics = [&]() {
        const double nowSeconds = Simulator::Now().GetSeconds();

        std::vector<std::string> row = {metricsRunLabel, relayMode, FormatDouble(nowSeconds, 3)};

        if (selectedRelayMode == RelayMode::SCF && houseCollectorApp)
        {
            const LayerAggregateStats sensorLayerNow = AggregateLayerStats(sensorStoreApps);
            const LayerAggregateStats level1LayerNow = AggregateLayerStats(level1StoreApps);
            const LayerAggregateStats level2LayerNow = AggregateLayerStats(level2StoreApps);
            const uint32_t coveredSensorsAtN1 = CountCoveredSensorsAtLevel1(level1StoreApps);
            const uint32_t deliveredSensorsAtHouse = houseCollectorApp->GetSensorsDeliveredToHouse();

            row.push_back(std::to_string(coveredSensorsAtN1));
            row.push_back(FormatDouble(100.0 * static_cast<double>(coveredSensorsAtN1) /
                                           static_cast<double>(sensorsPerField),
                                       4));
            row.push_back(std::to_string(houseCollectorApp->GetDeliveredSamples()));
            row.push_back(std::to_string(deliveredSensorsAtHouse));
            row.push_back(FormatDouble(100.0 * static_cast<double>(deliveredSensorsAtHouse) /
                                           static_cast<double>(sensorsPerField),
                                       4));
            row.push_back(FormatDouble(houseCollectorApp->GetAverageEndToEndDelay(), 6));
            row.push_back(FormatDouble(houseCollectorApp->GetCurrentAverageAoi(), 6));
            row.push_back(FormatDouble(houseCollectorApp->GetCurrentPeakAoi(), 6));
            row.push_back(std::to_string(sensorLayerNow.generated));
            row.push_back(std::to_string(sensorLayerNow.forwarded));
            row.push_back(std::to_string(sensorLayerNow.contactOpportunities));
            row.push_back(FormatDouble(ComputeCurrentAverageBufferOccupancy(sensorStoreApps), 6));
            row.push_back(std::to_string(sensorLayerNow.overflowDrops));
            row.push_back(std::to_string(sensorLayerNow.ttlDrops));
            row.push_back(std::to_string(level1LayerNow.received));
            row.push_back(std::to_string(level1LayerNow.forwarded));
            row.push_back(std::to_string(level1LayerNow.contactOpportunities));
            row.push_back(FormatDouble(ComputeCurrentAverageBufferOccupancy(level1StoreApps), 6));
            row.push_back(std::to_string(level1LayerNow.overflowDrops));
            row.push_back(std::to_string(level1LayerNow.ttlDrops));
            row.push_back(std::to_string(level2LayerNow.received));
            row.push_back(std::to_string(level2LayerNow.forwarded));
            row.push_back(std::to_string(level2LayerNow.contactOpportunities));
            row.push_back(FormatDouble(ComputeCurrentAverageBufferOccupancy(level2StoreApps), 6));
            row.push_back(std::to_string(level2LayerNow.overflowDrops));
            row.push_back(std::to_string(level2LayerNow.ttlDrops));

            for (uint32_t i = 0; i < nLevel1Clusters; ++i)
            {
                row.push_back(std::to_string(sensorsPerLevel1[i]));
                row.push_back(std::to_string(level1StoreApps[i]->GetUniqueSourceCount()));
                row.push_back(std::to_string(level1StoreApps[i]->GetReceivedSamples()));
                row.push_back(std::to_string(level1StoreApps[i]->GetForwardedSamples()));
                row.push_back(std::to_string(level1StoreApps[i]->GetCurrentBufferOccupancy()));
                row.push_back(std::to_string(level1StoreApps[i]->GetOverflowDrops()));
                row.push_back(std::to_string(level1StoreApps[i]->GetTtlDrops()));
            }

            for (uint32_t i = 0; i < nLevel2Clusters; ++i)
            {
                row.push_back(std::to_string(level1PerLevel2[i]));
                row.push_back(std::to_string(level2StoreApps[i]->GetUniqueSourceCount()));
                row.push_back(std::to_string(level2StoreApps[i]->GetReceivedSamples()));
                row.push_back(std::to_string(level2StoreApps[i]->GetForwardedSamples()));
                row.push_back(std::to_string(level2StoreApps[i]->GetCurrentBufferOccupancy()));
                row.push_back(std::to_string(level2StoreApps[i]->GetOverflowDrops()));
                row.push_back(std::to_string(level2StoreApps[i]->GetTtlDrops()));
            }
        }
        else
        {
            for (size_t i = 3; i < timeseriesHeaderFields.size(); ++i)
            {
                row.push_back("0");
            }
        }

        AppendCsvLine(timeseriesCsvFile, JoinCsvFields(row));

        if (nowSeconds + metricsSampleInterval <= simTime)
        {
            Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);
        }
    };

    Simulator::Schedule(Seconds(metricsSampleInterval), snapshotMetrics);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    const std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::vector<bool> sensorDelivered(sensorsPerField, false);
    std::vector<bool> level1LeadersDelivered(nLevel1Clusters, false);
    std::vector<bool> level2LeadersDelivered(nLevel2Clusters, false);

    const StageMetrics stage1Metrics =
        CollectStageMetrics(stats, classifier, sensorToL1Port, sensorIndexByIp, sensorDelivered);
    const StageMetrics stage2Metrics =
        CollectStageMetrics(stats, classifier, level1ToLevel2Port, level1LeaderIndexByIp, level1LeadersDelivered);
    const StageMetrics stage3Metrics =
        CollectStageMetrics(stats, classifier, level2ToHousePort, level2LeaderIndexByIp, level2LeadersDelivered);

    uint32_t coveredSensors = CountDeliveredSources(sensorDelivered);
    uint32_t deliveredLevel1Leaders = CountDeliveredSources(level1LeadersDelivered);
    uint32_t deliveredLevel2Leaders = CountDeliveredSources(level2LeadersDelivered);

    std::vector<uint32_t> coveredSensorsPerLevel1(nLevel1Clusters, 0);
    for (uint32_t i = 0; i < sensorDelivered.size(); ++i)
    {
        if (sensorDelivered[i])
        {
            coveredSensorsPerLevel1[sensorAssignedLevel1[i]]++;
        }
    }

    std::vector<uint32_t> deliveredLevel1PerLevel2(nLevel2Clusters, 0);
    for (uint32_t i = 0; i < level1LeadersDelivered.size(); ++i)
    {
        if (level1LeadersDelivered[i])
        {
            deliveredLevel1PerLevel2[level1AssignedLevel2[i]]++;
        }
    }

    std::cout << "\n===== RESULTADOS MANET JERARQUICA NIVELES 1 Y 2 =====\n";
    std::cout << "Modo de relevo: " << relayMode << "\n";
    std::cout << "Campo (m): " << fieldWidth << " x " << fieldHeight << "\n";
    std::cout << "Casa fija en: (" << housePosition.x << ", " << housePosition.y << ")\n";
    std::cout << "Nivel 1 -> clusters: " << nLevel1Clusters
              << ", nodos por cluster: " << agrobotsPerCluster << "\n";
    std::cout << "Nivel 2 -> clusters: " << nLevel2Clusters
              << ", nodos por cluster: " << level2NodesPerCluster << "\n";
    std::cout << "Sensores estaticos: " << sensorsPerField << "\n";

    if (selectedRelayMode == RelayMode::SCF)
    {
        std::cout << "Store-carry-forward por paquete activo: packetSize=" << packetSize
                  << " bytes, TTL=" << sampleTtl << " s, buffer=" << bufferCapacityPackets
                  << " muestras por nodo\n";
        std::cout << "Umbrales de contacto (m): S->N1=" << sensorContactRange
                  << ", N1->N2=" << level1ToLevel2ContactRange
                  << ", N2->Casa=" << level2ToHouseContactRange << "\n";
        std::cout << "Nota metodologica: en modo scf, level1ToLevel2PacketSize y "
                  << "level2ToHousePacketSize no se usan porque la misma muestra se transporta "
                  << "de extremo a extremo.\n";
    }

    std::cout << "\n[FlowMonitor Etapa 1] Sensores -> Lideres nivel 1\n";
    std::cout << "Sensores con recepcion en N1: " << coveredSensors << "/" << sensorsPerField << "\n";
    std::cout << "Cobertura N1 (%): "
              << (100.0 * static_cast<double>(coveredSensors) / static_cast<double>(sensorsPerField))
              << "\n";
    std::cout << "Paquetes TX: " << stage1Metrics.txPackets << "\n";
    std::cout << "Paquetes RX: " << stage1Metrics.rxPackets << "\n";
    std::cout << "PDR (%): " << ComputePdr(stage1Metrics) << "\n";
    std::cout << "Retardo promedio (s): " << ComputeAverageDelay(stage1Metrics) << "\n";

    std::cout << "\n[FlowMonitor Etapa 2] Lideres nivel 1 -> Lideres nivel 2\n";
    std::cout << "Lideres nivel 1 con recepcion en N2: " << deliveredLevel1Leaders << "/"
              << nLevel1Clusters << "\n";
    std::cout << "Paquetes TX: " << stage2Metrics.txPackets << "\n";
    std::cout << "Paquetes RX: " << stage2Metrics.rxPackets << "\n";
    std::cout << "PDR (%): " << ComputePdr(stage2Metrics) << "\n";
    std::cout << "Retardo promedio (s): " << ComputeAverageDelay(stage2Metrics) << "\n";

    std::cout << "\n[FlowMonitor Etapa 3] Lideres nivel 2 -> Casa\n";
    std::cout << "Lideres nivel 2 con recepcion en casa: " << deliveredLevel2Leaders << "/"
              << nLevel2Clusters << "\n";
    std::cout << "Paquetes TX: " << stage3Metrics.txPackets << "\n";
    std::cout << "Paquetes RX: " << stage3Metrics.rxPackets << "\n";
    std::cout << "PDR (%): " << ComputePdr(stage3Metrics) << "\n";
    std::cout << "Retardo promedio (s): " << ComputeAverageDelay(stage3Metrics) << "\n";

    if (selectedRelayMode == RelayMode::SCF && houseCollectorApp)
    {
        const LayerAggregateStats sensorLayerStats = AggregateLayerStats(sensorStoreApps);
        const LayerAggregateStats level1LayerStats = AggregateLayerStats(level1StoreApps);
        const LayerAggregateStats level2LayerStats = AggregateLayerStats(level2StoreApps);

        std::cout << "\n[SCF extremo a extremo] Sensores -> Casa\n";
        std::cout << "Sensores con al menos una muestra entregada a casa: "
                  << houseCollectorApp->GetSensorsDeliveredToHouse() << "/" << sensorsPerField
                  << "\n";
        std::cout << "Cobertura a casa (%): "
                  << (100.0 * static_cast<double>(houseCollectorApp->GetSensorsDeliveredToHouse()) /
                      static_cast<double>(sensorsPerField))
                  << "\n";
        std::cout << "Muestras unicas recibidas en casa: " << houseCollectorApp->GetDeliveredSamples()
                  << "\n";
        std::cout << "Muestras duplicadas en casa: " << houseCollectorApp->GetDuplicateSamples()
                  << "\n";
        std::cout << "Muestras obsoletas recibidas en casa: " << houseCollectorApp->GetStaleSamples()
                  << "\n";
        std::cout << "Delay extremo a extremo promedio (s): "
                  << houseCollectorApp->GetAverageEndToEndDelay() << "\n";
        std::cout << "Delay extremo a extremo minimo (s): "
                  << houseCollectorApp->GetMinEndToEndDelay() << "\n";
        std::cout << "Delay extremo a extremo maximo (s): "
                  << houseCollectorApp->GetMaxEndToEndDelay() << "\n";
        std::cout << "Age of Information promedio (s): " << houseCollectorApp->GetAverageAoi()
                  << "\n";
        std::cout << "Peak Age of Information (s): " << houseCollectorApp->GetPeakAoi() << "\n";

        std::cout << "\n[SCF buffers] Sensores\n";
        std::cout << "Muestras generadas: " << sensorLayerStats.generated << "\n";
        std::cout << "Muestras reenviadas: " << sensorLayerStats.forwarded << "\n";
        std::cout << "Oportunidades de contacto: " << sensorLayerStats.contactOpportunities << "\n";
        std::cout << "Drops por overflow: " << sensorLayerStats.overflowDrops << "\n";
        std::cout << "Drops por TTL: " << sensorLayerStats.ttlDrops << "\n";
        std::cout << "Ocupacion promedio de buffer: " << ComputeAverageOccupancy(sensorLayerStats)
                  << "\n";
        std::cout << "Ocupacion maxima de buffer: " << sensorLayerStats.maxOccupancy << "\n";
        std::cout << "Carry time promedio (s): " << ComputeAverageCarryTime(sensorLayerStats) << "\n";

        std::cout << "\n[SCF buffers] Relays nivel 1\n";
        std::cout << "Muestras recibidas: " << level1LayerStats.received << "\n";
        std::cout << "Muestras duplicadas descartadas: " << level1LayerStats.duplicates << "\n";
        std::cout << "Muestras reenviadas: " << level1LayerStats.forwarded << "\n";
        std::cout << "Oportunidades de contacto: " << level1LayerStats.contactOpportunities << "\n";
        std::cout << "Drops por overflow: " << level1LayerStats.overflowDrops << "\n";
        std::cout << "Drops por TTL: " << level1LayerStats.ttlDrops << "\n";
        std::cout << "Ocupacion promedio de buffer: " << ComputeAverageOccupancy(level1LayerStats)
                  << "\n";
        std::cout << "Ocupacion maxima de buffer: " << level1LayerStats.maxOccupancy << "\n";
        std::cout << "Carry time promedio (s): " << ComputeAverageCarryTime(level1LayerStats)
                  << "\n";

        std::cout << "\n[SCF buffers] Relays nivel 2\n";
        std::cout << "Muestras recibidas: " << level2LayerStats.received << "\n";
        std::cout << "Muestras duplicadas descartadas: " << level2LayerStats.duplicates << "\n";
        std::cout << "Muestras reenviadas: " << level2LayerStats.forwarded << "\n";
        std::cout << "Oportunidades de contacto: " << level2LayerStats.contactOpportunities << "\n";
        std::cout << "Drops por overflow: " << level2LayerStats.overflowDrops << "\n";
        std::cout << "Drops por TTL: " << level2LayerStats.ttlDrops << "\n";
        std::cout << "Ocupacion promedio de buffer: " << ComputeAverageOccupancy(level2LayerStats)
                  << "\n";
        std::cout << "Ocupacion maxima de buffer: " << level2LayerStats.maxOccupancy << "\n";
        std::cout << "Carry time promedio (s): " << ComputeAverageCarryTime(level2LayerStats)
                  << "\n";
    }
    else
    {
        std::cout << "\n[SCF] No activo. En modo baseline solo se reportan metricas de FlowMonitor.\n";
    }

    std::cout << "\nAsignaciones nivel 1\n";
    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        std::cout << "Cluster N1 " << (i + 1)
                  << " -> sensores asignados: " << sensorsPerLevel1[i]
                  << ", sensores con recepcion en N1: " << coveredSensorsPerLevel1[i]
                  << ", cluster N2 destino: " << (level1AssignedLevel2[i] + 1) << "\n";
    }

    std::cout << "\nAsignaciones nivel 2\n";
    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        std::cout << "Cluster N2 " << (i + 1)
                  << " -> lideres N1 asignados: " << level1PerLevel2[i]
                  << ", lideres N1 con recepcion en N2: " << deliveredLevel1PerLevel2[i] << "\n";
    }

    std::cout << "FlowMonitor XML: " << flowmonFile << "\n";

    monitor->SerializeToXmlFile(flowmonFile, true, true);

    std::vector<std::string> summaryHeaderFields = {"run_label",
                                                    "relay_mode",
                                                    "rng_run",
                                                    "sim_time_s",
                                                    "field_width_m",
                                                    "field_height_m",
                                                    "sensors_per_field",
                                                    "n1_clusters",
                                                    "agrobots_per_cluster",
                                                    "n2_clusters",
                                                    "level2_nodes_per_cluster",
                                                    "metrics_sample_interval_s",
                                                    "stage1_tx",
                                                    "stage1_rx",
                                                    "stage1_pdr_pct",
                                                    "stage1_avg_delay_s",
                                                    "stage2_tx",
                                                    "stage2_rx",
                                                    "stage2_pdr_pct",
                                                    "stage2_avg_delay_s",
                                                    "stage3_tx",
                                                    "stage3_rx",
                                                    "stage3_pdr_pct",
                                                    "stage3_avg_delay_s",
                                                    "house_delivered_samples",
                                                    "house_delivered_sensors",
                                                    "house_coverage_pct",
                                                    "house_e2e_delay_avg_s",
                                                    "house_e2e_delay_min_s",
                                                    "house_e2e_delay_max_s",
                                                    "house_aoi_avg_s",
                                                    "house_aoi_peak_s",
                                                    "house_duplicates",
                                                    "house_stale_samples",
                                                    "sensor_generated",
                                                    "sensor_forwarded",
                                                    "sensor_contacts",
                                                    "sensor_overflow_drops",
                                                    "sensor_ttl_drops",
                                                    "sensor_avg_buffer_occ",
                                                    "sensor_max_buffer_occ",
                                                    "sensor_avg_carry_s",
                                                    "n1_received_samples",
                                                    "n1_duplicates",
                                                    "n1_forwarded_samples",
                                                    "n1_contacts",
                                                    "n1_overflow_drops",
                                                    "n1_ttl_drops",
                                                    "n1_avg_buffer_occ",
                                                    "n1_max_buffer_occ",
                                                    "n1_avg_carry_s",
                                                    "n2_received_samples",
                                                    "n2_duplicates",
                                                    "n2_forwarded_samples",
                                                    "n2_contacts",
                                                    "n2_overflow_drops",
                                                    "n2_ttl_drops",
                                                    "n2_avg_buffer_occ",
                                                    "n2_max_buffer_occ",
                                                    "n2_avg_carry_s"};

    for (uint32_t i = 0; i < nLevel1Clusters; ++i)
    {
        const std::string prefix = "n1_cluster_" + std::to_string(i + 1) + "_";
        summaryHeaderFields.push_back(prefix + "assigned_sensors");
        summaryHeaderFields.push_back(prefix + "covered_sensors_final");
        summaryHeaderFields.push_back(prefix + "received_samples");
        summaryHeaderFields.push_back(prefix + "forwarded_samples");
        summaryHeaderFields.push_back(prefix + "avg_buffer_occ");
        summaryHeaderFields.push_back(prefix + "max_buffer_occ");
        summaryHeaderFields.push_back(prefix + "overflow_drops");
        summaryHeaderFields.push_back(prefix + "ttl_drops");
        summaryHeaderFields.push_back(prefix + "avg_carry_s");
    }

    for (uint32_t i = 0; i < nLevel2Clusters; ++i)
    {
        const std::string prefix = "n2_cluster_" + std::to_string(i + 1) + "_";
        summaryHeaderFields.push_back(prefix + "assigned_n1_leaders");
        summaryHeaderFields.push_back(prefix + "unique_sensor_sources_final");
        summaryHeaderFields.push_back(prefix + "received_samples");
        summaryHeaderFields.push_back(prefix + "forwarded_samples");
        summaryHeaderFields.push_back(prefix + "avg_buffer_occ");
        summaryHeaderFields.push_back(prefix + "max_buffer_occ");
        summaryHeaderFields.push_back(prefix + "overflow_drops");
        summaryHeaderFields.push_back(prefix + "ttl_drops");
        summaryHeaderFields.push_back(prefix + "avg_carry_s");
    }

    EnsureCsvHeader(summaryCsvFile, JoinCsvFields(summaryHeaderFields));

    std::vector<std::string> summaryRow = {metricsRunLabel,
                                           relayMode,
                                           std::to_string(rngRun),
                                           FormatDouble(simTime, 3),
                                           FormatDouble(fieldWidth, 3),
                                           FormatDouble(fieldHeight, 3),
                                           std::to_string(sensorsPerField),
                                           std::to_string(nLevel1Clusters),
                                           std::to_string(agrobotsPerCluster),
                                           std::to_string(nLevel2Clusters),
                                           std::to_string(level2NodesPerCluster),
                                           FormatDouble(metricsSampleInterval, 3),
                                           std::to_string(stage1Metrics.txPackets),
                                           std::to_string(stage1Metrics.rxPackets),
                                           FormatDouble(ComputePdr(stage1Metrics), 6),
                                           FormatDouble(ComputeAverageDelay(stage1Metrics), 6),
                                           std::to_string(stage2Metrics.txPackets),
                                           std::to_string(stage2Metrics.rxPackets),
                                           FormatDouble(ComputePdr(stage2Metrics), 6),
                                           FormatDouble(ComputeAverageDelay(stage2Metrics), 6),
                                           std::to_string(stage3Metrics.txPackets),
                                           std::to_string(stage3Metrics.rxPackets),
                                           FormatDouble(ComputePdr(stage3Metrics), 6),
                                           FormatDouble(ComputeAverageDelay(stage3Metrics), 6)};

    if (selectedRelayMode == RelayMode::SCF && houseCollectorApp)
    {
        const LayerAggregateStats sensorLayerStats = AggregateLayerStats(sensorStoreApps);
        const LayerAggregateStats level1LayerStats = AggregateLayerStats(level1StoreApps);
        const LayerAggregateStats level2LayerStats = AggregateLayerStats(level2StoreApps);

        summaryRow.push_back(std::to_string(houseCollectorApp->GetDeliveredSamples()));
        summaryRow.push_back(std::to_string(houseCollectorApp->GetSensorsDeliveredToHouse()));
        summaryRow.push_back(FormatDouble(100.0 * static_cast<double>(houseCollectorApp->GetSensorsDeliveredToHouse()) /
                                              static_cast<double>(sensorsPerField),
                                          6));
        summaryRow.push_back(FormatDouble(houseCollectorApp->GetAverageEndToEndDelay(), 6));
        summaryRow.push_back(FormatDouble(houseCollectorApp->GetMinEndToEndDelay(), 6));
        summaryRow.push_back(FormatDouble(houseCollectorApp->GetMaxEndToEndDelay(), 6));
        summaryRow.push_back(FormatDouble(houseCollectorApp->GetAverageAoi(), 6));
        summaryRow.push_back(FormatDouble(houseCollectorApp->GetPeakAoi(), 6));
        summaryRow.push_back(std::to_string(houseCollectorApp->GetDuplicateSamples()));
        summaryRow.push_back(std::to_string(houseCollectorApp->GetStaleSamples()));

        summaryRow.push_back(std::to_string(sensorLayerStats.generated));
        summaryRow.push_back(std::to_string(sensorLayerStats.forwarded));
        summaryRow.push_back(std::to_string(sensorLayerStats.contactOpportunities));
        summaryRow.push_back(std::to_string(sensorLayerStats.overflowDrops));
        summaryRow.push_back(std::to_string(sensorLayerStats.ttlDrops));
        summaryRow.push_back(FormatDouble(ComputeAverageOccupancy(sensorLayerStats), 6));
        summaryRow.push_back(std::to_string(sensorLayerStats.maxOccupancy));
        summaryRow.push_back(FormatDouble(ComputeAverageCarryTime(sensorLayerStats), 6));

        summaryRow.push_back(std::to_string(level1LayerStats.received));
        summaryRow.push_back(std::to_string(level1LayerStats.duplicates));
        summaryRow.push_back(std::to_string(level1LayerStats.forwarded));
        summaryRow.push_back(std::to_string(level1LayerStats.contactOpportunities));
        summaryRow.push_back(std::to_string(level1LayerStats.overflowDrops));
        summaryRow.push_back(std::to_string(level1LayerStats.ttlDrops));
        summaryRow.push_back(FormatDouble(ComputeAverageOccupancy(level1LayerStats), 6));
        summaryRow.push_back(std::to_string(level1LayerStats.maxOccupancy));
        summaryRow.push_back(FormatDouble(ComputeAverageCarryTime(level1LayerStats), 6));

        summaryRow.push_back(std::to_string(level2LayerStats.received));
        summaryRow.push_back(std::to_string(level2LayerStats.duplicates));
        summaryRow.push_back(std::to_string(level2LayerStats.forwarded));
        summaryRow.push_back(std::to_string(level2LayerStats.contactOpportunities));
        summaryRow.push_back(std::to_string(level2LayerStats.overflowDrops));
        summaryRow.push_back(std::to_string(level2LayerStats.ttlDrops));
        summaryRow.push_back(FormatDouble(ComputeAverageOccupancy(level2LayerStats), 6));
        summaryRow.push_back(std::to_string(level2LayerStats.maxOccupancy));
        summaryRow.push_back(FormatDouble(ComputeAverageCarryTime(level2LayerStats), 6));

        for (uint32_t i = 0; i < nLevel1Clusters; ++i)
        {
            summaryRow.push_back(std::to_string(sensorsPerLevel1[i]));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetUniqueSourceCount()));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetReceivedSamples()));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetForwardedSamples()));
            summaryRow.push_back(FormatDouble(level1StoreApps[i]->GetAverageOccupancy(), 6));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetMaxOccupancy()));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetOverflowDrops()));
            summaryRow.push_back(std::to_string(level1StoreApps[i]->GetTtlDrops()));
            summaryRow.push_back(FormatDouble(level1StoreApps[i]->GetCarryTimeSumSeconds() /
                                                  std::max<uint64_t>(1, level1StoreApps[i]->GetCarrySamples()),
                                              6));
        }

        for (uint32_t i = 0; i < nLevel2Clusters; ++i)
        {
            summaryRow.push_back(std::to_string(level1PerLevel2[i]));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetUniqueSourceCount()));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetReceivedSamples()));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetForwardedSamples()));
            summaryRow.push_back(FormatDouble(level2StoreApps[i]->GetAverageOccupancy(), 6));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetMaxOccupancy()));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetOverflowDrops()));
            summaryRow.push_back(std::to_string(level2StoreApps[i]->GetTtlDrops()));
            summaryRow.push_back(FormatDouble(level2StoreApps[i]->GetCarryTimeSumSeconds() /
                                                  std::max<uint64_t>(1, level2StoreApps[i]->GetCarrySamples()),
                                              6));
        }
    }
    else
    {
        for (size_t i = 24; i < summaryHeaderFields.size(); ++i)
        {
            summaryRow.push_back("0");
        }
    }

    AppendCsvLine(summaryCsvFile, JoinCsvFields(summaryRow));

    Simulator::Destroy();
    return 0;
}
