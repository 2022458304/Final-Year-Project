#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrDemo");

int main(int argc, char* argv[])
{
    uint16_t gNbNum = 3; // Number of gNBs
    uint16_t ueNum = 5; // Number of UEs
    bool logging = false;
    bool doubleOperationalBand = true;

    uint32_t udpPacketSizeBe = 512; // Smaller packet size for low latency
    uint32_t lambdaBe = 10000; // Reduce data rate

    // Simulation time and application start time
    double simTime = 60.0;
    double udpAppStartTime = 0.1;

    // Frequency parameters
    uint16_t numerologyBwp1 = 3; // Adjusted numerology for low latency
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 400e6; // Increased bandwidth
    uint16_t numerologyBwp2 = 2;
    double centralFrequencyBand2 = 28.2e9;
    double bandwidthBand2 = 400e6; // Increased bandwidth
    double totalTxPower = 55; // Transmission power

    std::string simTag = "default";
    std::string outputDir = "./";
    
    // Check for invalid frequency values
    NS_ABORT_IF(centralFrequencyBand1 < 0.5e9 || centralFrequencyBand1 > 400e9);
    NS_ABORT_IF(centralFrequencyBand2 < 0.5e9 || centralFrequencyBand2 > 400e9);

    // Enable logging for specific components if logging is enabled
    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
    }
    
    // Set default max TX buffer size for LteRlcUm
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // Set random stream and create a grid scenario with 1 row and gNbNum columns
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);

    // Set horizontal and vertical distances between gNBs
    gridScenario.SetHorizontalBsDistance(100.0); // Distance between gNBs
    gridScenario.SetVerticalBsDistance(10.0);
    gridScenario.SetBsHeight(10);
    gridScenario.SetUtHeight(1.5);

    // Set sectorization and number of gNBs and UEs
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNbNum);
    gridScenario.SetUtNumber(ueNum);

    // Assign streams and create the scenario
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    // Create the position and the mobility for the base stations (gNBs)
    MobilityHelper bsMobility;
    bsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    bsMobility.Install(gridScenario.GetBaseStations());
    gridScenario.GetBaseStations().Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(30.0, 50.0, 10.0));
    gridScenario.GetBaseStations().Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(50.0, 50.0, 10.0));
    gridScenario.GetBaseStations().Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(70.0, 50.0, 10.0)); // Added third gNB

    // Create node container for low latency traffic
    NodeContainer ueLowLatencyContainer;
    MobilityHelper ueMobility;
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);
        ueLowLatencyContainer.Add(ue);
    }
    
    // Set up mobility for user terminals and position
    ueMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                    "MinX", DoubleValue(30.0),
                                    "MinY", DoubleValue(60.0),
                                    "DeltaX", DoubleValue(10.0),
                                    "DeltaY", DoubleValue(10.0),
                                    "GridWidth", UintegerValue(5),
                                    "LayoutType", StringValue("RowFirst"));

    ueMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(0, 200, 0, 100)),
                                "Speed", StringValue("ns3::ConstantRandomVariable[Constant=2]"),
                                "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));

    ueMobility.Install(gridScenario.GetUserTerminals());
    
    NS_LOG_INFO("Creating " << gridScenario.GetUserTerminals().GetN() << " user terminals and "
                            << gridScenario.GetBaseStations().GetN() << " gNBs");

    // Create the EPC network environment (PGW, SGW, and MME)
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Create bandwidth part info vector
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    // Create bandwidth 1 configurations
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1, bandwidthBand1, numCcPerBand, BandwidthPartInfo::UMi_StreetCanyon);
    // Create bandwidth 2 configurations
    CcBwpCreator::SimpleOperationBandConf bandConf2(centralFrequencyBand2, bandwidthBand2, numCcPerBand, BandwidthPartInfo::UMi_StreetCanyon);

    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf2);

    // Set up channel model and pathloss attributes
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Initialize operation band 1
    nrHelper->InitializeOperationBand(&band1);

    double x = pow(20, totalTxPower / 5);
    double totalBandwidth = bandwidthBand1;

    if (doubleOperationalBand)
    {
        // Initialize operation band 2 if double operational band is enabled
        nrHelper->InitializeOperationBand(&band2);
        totalBandwidth += bandwidthBand2;
        allBwps = CcBwpCreator::GetAllBwps({band1, band2});
    }
    else
    {
        allBwps = CcBwpCreator::GetAllBwps({band1});
    }

    // Enable packet checking and printing
    Packet::EnableChecking();
    Packet::EnablePrinting();

    // Set beamforming method to Direct Path Beamforming
    idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Set UE antenna attributes
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Set gNB antenna attributes
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Set BWP ID for voice traffic
    uint32_t bwpIdForLowLatency = 0;
    if (doubleOperationalBand)
    {
        bwpIdForLowLatency = 1;
    }

    // Set BWP manager algorithm attributes for voice traffic
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLatency));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLatency));

    // Set the scheduler type to Round Robin
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    
    // Install gNB and UE devices
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueLowLatencyNetDev = nrHelper->InstallUeDevice(ueLowLatencyContainer, allBwps);

    // Assign streams to devices
    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueLowLatencyNetDev, randomStream);

    // Configure gNB PHY attributes
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x)));
    nrHelper->GetGnbPhy(enbNetDev.Get(1), 0)->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    nrHelper->GetGnbPhy(enbNetDev.Get(1), 0)->SetAttribute("TxPower", DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x)));
    nrHelper->GetGnbPhy(enbNetDev.Get(2), 0)->SetAttribute("Numerology", UintegerValue(numerologyBwp1)); // Added third gNB configuration
    nrHelper->GetGnbPhy(enbNetDev.Get(2), 0)->SetAttribute("TxPower", DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x))); // Adjusted for third gNB

    // Configure gNB PHY attributes for second band (if double operational band is enabled)
    if (doubleOperationalBand)
    {
        nrHelper->GetGnbPhy(enbNetDev.Get(0), 1)->SetAttribute("Numerology", UintegerValue(numerologyBwp2));
        nrHelper->GetGnbPhy(enbNetDev.Get(0), 1)->SetTxPower(10 * log10((bandwidthBand2 / totalBandwidth) * x));
        nrHelper->GetGnbPhy(enbNetDev.Get(1), 1)->SetAttribute("Numerology", UintegerValue(numerologyBwp2));
        nrHelper->GetGnbPhy(enbNetDev.Get(1), 1)->SetTxPower(10 * log10((bandwidthBand2 / totalBandwidth) * x));
        nrHelper->GetGnbPhy(enbNetDev.Get(2), 1)->SetAttribute("Numerology", UintegerValue(numerologyBwp2)); // Added third gNB configuration for second band
        nrHelper->GetGnbPhy(enbNetDev.Get(2), 1)->SetTxPower(10 * log10((bandwidthBand2 / totalBandwidth) * x)); // Adjusted for third gNB
    }

    // Update device configurations
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueLowLatencyNetDev.Begin(); it != ueLowLatencyNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // Set up PGW node
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    MobilityHelper pgwMobility;
    Ptr<ListPositionAllocator> positionAllocPgw = CreateObject<ListPositionAllocator>();
    positionAllocPgw->Add(Vector(70.0, 0.0, 1.5));
    pgwMobility.SetPositionAllocator(positionAllocPgw);
    pgwMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    pgwMobility.Install(pgw);

    // Set up SGW node
    Ptr<Node> sgw = epcHelper->GetSgwNode();
    MobilityHelper sgwMobility;
    Ptr<ListPositionAllocator> positionAllocSgw = CreateObject<ListPositionAllocator>();
    positionAllocSgw->Add(Vector(50.0, 0.0, 1.5));
    sgwMobility.SetPositionAllocator(positionAllocSgw);
    sgwMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    sgwMobility.Install(sgw);

    // Set up MME node
    Ptr<Node> mme = epcHelper->GetMmeNode();
    MobilityHelper MmeMobility;
    Ptr<ListPositionAllocator> positionAllocMme = CreateObject<ListPositionAllocator>();
    positionAllocMme->Add(Vector(40.0, 0.0, 1.5));
    MmeMobility.SetPositionAllocator(positionAllocMme);
    MmeMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    MmeMobility.Install(mme);

    // Create a remote host node
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // Create Ip stack and install it on remote host.
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Set up remote host mobility
    MobilityHelper remoteHostMobility;
    Ptr<ListPositionAllocator> positionAllocRemoteHost = CreateObject<ListPositionAllocator>();
    positionAllocRemoteHost->Add(Vector(90.0, 0.0, 1.5));
    remoteHostMobility.SetPositionAllocator(positionAllocRemoteHost);
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(remoteHost);

    // Create a point to point connection between PGW and RH
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // IP address assignment
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    // Set the base for the ip address
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(gridScenario.GetUserTerminals());

    // Assign IP addresses to UEs
    Ipv4InterfaceContainer ueLowLatencyIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLowLatencyNetDev));

    // Set up default routes for UEs
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(gridScenario.GetUserTerminals().Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach UEs to the gNBs
    for (uint32_t i = 0; i < ueLowLatencyNetDev.GetN(); ++i)
    {
        nrHelper->AttachToEnb(ueLowLatencyNetDev.Get(i), enbNetDev.Get(i % gNbNum)); // Alternate attachment to gNBs
    }

    uint16_t dlPortLowLatency = 1236;

    ApplicationContainer serverApps;
    UdpServerHelper dlPacketSinkLowLatency(dlPortLowLatency);
    serverApps.Add(dlPacketSinkLowLatency.Install(ueLowLatencyContainer));

    UdpClientHelper dlClientLowLatency;
    dlClientLowLatency.SetAttribute("RemotePort", UintegerValue(dlPortLowLatency));
    dlClientLowLatency.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientLowLatency.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));
    dlClientLowLatency.SetAttribute("Interval", TimeValue(Seconds(5000.0 / lambdaBe))); // Higher data rate for low latency

    // The bearer that will carry low latency traffic
    EpsBearer lowLatBearer(EpsBearer::NGBR_LOW_LAT_EMBB);

 // Create a TFT (Traffic Flow Template) for low latency traffic
Ptr<EpcTft> lowLatencyTft = Create<EpcTft>();
EpcTft::PacketFilter dlpfLowLatency;
dlpfLowLatency.localPortStart = dlPortLowLatency;
dlpfLowLatency.localPortEnd = dlPortLowLatency;
lowLatencyTft->Add(dlpfLowLatency);

// Create a container for client applications
ApplicationContainer clientApps;

// Install client applications on each UE (User Equipment) node
for (uint32_t i = 0; i < ueLowLatencyContainer.GetN(); ++i)
{
    // Get the UE node, device, and address
    Ptr<Node> ue = ueLowLatencyContainer.Get(i);
    Ptr<NetDevice> ueDevice = ueLowLatencyNetDev.Get(i);
    Address ueAddress = ueLowLatencyIpIface.GetAddress(i);

    // Set the remote address for the client application
    dlClientLowLatency.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    clientApps.Add(dlClientLowLatency.Install(remoteHost));

    // Activate the dedicated EPS bearer for the UE
    nrHelper->ActivateDedicatedEpsBearer(ueDevice, lowLatBearer, lowLatencyTft);
}

// Start and stop the server and client applications
serverApps.Start(Seconds(udpAppStartTime));
clientApps.Start(Seconds(udpAppStartTime));
serverApps.Stop(Seconds(simTime));
clientApps.Stop(Seconds(simTime));

// Create a flow monitor to track network flows
FlowMonitorHelper flowmonHelper;
NodeContainer endpointNodes;
endpointNodes.Add(remoteHost);
endpointNodes.Add(gridScenario.GetUserTerminals());

Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

// Create an animation interface to visualize the simulation
AnimationInterface anim("5G_PF_LowLatency.xml");

// Update node descriptions and colors for the animation
for (uint32_t i = 0; i < ueLowLatencyContainer.GetN(); ++i)
{
    std::stringstream ss;
    ss << "UE-ll" << i + 1;
    anim.UpdateNodeDescription(ueLowLatencyContainer.Get(i), ss.str());
    anim.UpdateNodeColor(ueLowLatencyContainer.Get(i), 0, 255, 0);
}

// Update node descriptions and colors for the animation
for (uint32_t i = 0; i < gridScenario.GetBaseStations().GetN(); ++i)
{
    std::stringstream ss;
    ss << "gNB-" << i + 1;
    anim.UpdateNodeDescription(gridScenario.GetBaseStations().Get(i), ss.str());
    anim.UpdateNodeColor(gridScenario.GetBaseStations().Get(i), 255, 0, 0);
}

// Update node descriptions and colors for the animation
anim.UpdateNodeDescription(pgw, "PGW");
anim.UpdateNodeColor(pgw, 255, 255, 0);

anim.UpdateNodeDescription(sgw, "SGW");
anim.UpdateNodeColor(sgw, 255, 250, 0);

anim.UpdateNodeDescription(mme, "MME");
anim.UpdateNodeColor(mme, 255, 250, 0);

anim.UpdateNodeDescription(remoteHost, "RH");
anim.UpdateNodeColor(remoteHost, 0, 0, 255);

// Enable packet metadata for the animation
anim.EnablePacketMetadata(true);

// Stop the simulation at the specified time
Simulator::Stop(Seconds(simTime));
Simulator::Run();

    // Check for lost packets in the flow monitor
monitor->CheckForLostPackets();

// Get the flow classifier and flow stats
Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

// Initialize variables to calculate overall statistics
double totalRxBytes = 0.0;
double totalDelay = 0.0;
double totalLostPackets = 0.0;
uint32_t totalRxPackets = 0;
uint32_t totalTxPackets = 0;
uint32_t totalFlows = 0;

// Calculate the flow duration
double flowDuration = (Seconds(simTime) - Seconds(udpAppStartTime)).GetSeconds();

// Iterate over each flow in the flow stats
for (auto i = stats.begin(); i != stats.end(); ++i)
{
    // Get the five-tuple for the current flow
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);

    // Print flow information
    std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
              << t.destinationAddress << ":" << t.destinationPort << ") proto "
              << (t.protocol == 6 ? "TCP" : "UDP") << "\n";
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
              << " Mbps\n";
    std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";

    // Calculate and print flow statistics if there are received packets
    if (i->second.rxPackets > 0)
    {
        double throughput = i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
        double delay = 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
        double lossRate = (i->second.txPackets - i->second.rxPackets) * 100.0 / i->second.txPackets;

        std::cout << "  Throughput: " << throughput << " Mbps\n";
        std::cout << "  Mean delay:  " << delay << " ms\n";
        std::cout << "  Packet loss rate:  " << lossRate << " %\n";

        // Update overall statistics
        totalRxBytes += i->second.rxBytes;
        totalDelay += i->second.delaySum.GetSeconds();
        totalLostPackets += (i->second.txPackets - i->second.rxPackets);
        totalRxPackets += i->second.rxPackets;
        totalTxPackets += i->second.txPackets;
        totalFlows++;
    }
    else
    {
        std::cout << "  Throughput:  0 Mbps\n";
        std::cout << "  Mean delay:  0 ms\n";
        std::cout << "  Packet loss rate:  100 %\n";
    }
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
}
    // Calculate overall statistics
double meanThroughput = totalRxBytes * 8.0 / (flowDuration * totalFlows) / 1000 / 1000;
double meanDelay = totalDelay / totalRxPackets * 1000;
double packetLossRate = totalLostPackets * 100.0 / totalTxPackets;

// Calculate fairness index if there are multiple flows
double fairnessIndex = 0.0;
if (totalFlows > 1)
{
    double sumThroughput = 0.0;
    double sumThroughputSq = 0.0;
    for (auto i = stats.begin(); i!= stats.end(); ++i)
    {
        if (i->second.rxPackets > 0)
        {
            double throughput = i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            sumThroughput += throughput;
            sumThroughputSq += throughput * throughput;
        }
    }
    fairnessIndex = (sumThroughput * sumThroughput) / (totalFlows * sumThroughputSq);
}

// Print overall statistics
std::cout << "\n\n  Mean throughput: " << meanThroughput << " Mbps\n";
std::cout << "  Mean delay: " << meanDelay << " ms\n";
std::cout << "  Packet loss rate: " << packetLossRate << " %\n";
std::cout << "  Fairness index: " << fairnessIndex << "\n";

Simulator::Destroy();

std::cout << "Simulation end time: " << Simulator::Now().GetSeconds() << " seconds" << std::endl;

return EXIT_SUCCESS;
}
