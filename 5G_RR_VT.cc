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

// Define a log component for the simulation
NS_LOG_COMPONENT_DEFINE("CttcNrDemo");

int main(int argc, char* argv[])
{
    // Number of gNBs (increased to 3)
    uint16_t gNbNum = 3; 
    // Number of UEs
    uint16_t ueNum = 5; 
    // Enable logging (set to false by default)
    bool logging = false;
    // Enable double operational band (set to true by default)
    bool doubleOperationalBand = true;

    // UDP packet size and data rate
    uint32_t udpPacketSizeBe = 1024;
    uint32_t lambdaBe = 10000; // Reduced data rate

    // Simulation time and application start time
    double simTime = 60.0;
    double udpAppStartTime = 0.1;

    // Frequency parameters
    uint16_t numerologyBwp1 = 4;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 100e6; // Bandwidth
    uint16_t numerologyBwp2 = 2;
    double centralFrequencyBand2 = 28.2e9;
    double bandwidthBand2 = 100e6; // Bandwidth
    double totalTxPower = 55; // Transmission power

    // Simulation tag and output directory
    std::string simTag = "default";
    std::string outputDir = "./";

    // Check for invalid frequency values
    NS_ABORT_IF(centralFrequencyBand1 < 0.5e9 || centralFrequencyBand1 > 100e9);
    NS_ABORT_IF(centralFrequencyBand2 < 0.5e9 || centralFrequencyBand2 > 100e9);

    // Enable logging for specific components if logging is enabled
    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
    }

    // Set default max TX buffer size for LteRlcUm
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // Set random stream
    int64_t randomStream = 1;

    // Create a grid scenario with 1 row and gNbNum columns
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);

    // Set horizontal and vertical distances between gNBs
    gridScenario.SetHorizontalBsDistance(100.0); // Increased distance between gNBs
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

    // Create position and mobility for base stations (gNBs)
    MobilityHelper bsMobility;
    bsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    bsMobility.Install(gridScenario.GetBaseStations());
    gridScenario.GetBaseStations().Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(30.0, 50.0, 10.0));
    gridScenario.GetBaseStations().Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(50.0, 50.0, 10.0));
    gridScenario.GetBaseStations().Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(70.0, 50.0, 10.0)); // Added third gNB

    // Create node container for voice
NodeContainer ueVoiceContainer;
MobilityHelper ueMobility;
for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
{
    Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);
    ueVoiceContainer.Add(ue);                               
}

// Set up mobility for user terminals
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

// Set S1u link delay to 0ms
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
uint32_t bwpIdForVoice = 0;
if (doubleOperationalBand)
{
    bwpIdForVoice = 1;
}

// Set BWP manager algorithm attributes for voice traffic
nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForVoice));
nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForVoice));

// Set scheduler type to Round Robin 
nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

// Install gNB and UE devices
NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
NetDeviceContainer ueVoiceNetDev = nrHelper->InstallUeDevice(ueVoiceContainer, allBwps);

// Assign streams to devices
randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
randomStream += nrHelper->AssignStreams(ueVoiceNetDev, randomStream);

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

for (auto it = ueVoiceNetDev.Begin(); it != ueVoiceNetDev.End(); ++it)
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

// Install Internet stack on remote host
InternetStackHelper internet;
internet.Install(remoteHostContainer);

// Set up remote host mobility
MobilityHelper remoteHostMobility;
Ptr<ListPositionAllocator> positionAllocRemoteHost = CreateObject<ListPositionAllocator>();
positionAllocRemoteHost->Add(Vector(90.0, 0.0, 1.5));
remoteHostMobility.SetPositionAllocator(positionAllocRemoteHost);
remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
remoteHostMobility.Install(remoteHost);

// Create a point-to-point connection between PGW and remote host
PointToPointHelper p2ph;
p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

// Assign IP addresses to the point-to-point connection
Ipv4AddressHelper ipv4h;
Ipv4StaticRoutingHelper ipv4RoutingHelper;

ipv4h.SetBase("1.0.0.0", "255.0.0.0");
Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

// Install Internet stack on user terminals
internet.Install(gridScenario.GetUserTerminals());

// Assign IP addresses to UEs
Ipv4InterfaceContainer ueVoiceIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueVoiceNetDev));

// Set up default routes for UEs
for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
{
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(gridScenario.GetUserTerminals().Get(j)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
}

// Attach UEs to gNBs
for (uint32_t i = 0; i < ueVoiceNetDev.GetN(); ++i)
{
    nrHelper->AttachToEnb(ueVoiceNetDev.Get(i), enbNetDev.Get(i % gNbNum)); // Alternate attachment to gNBs
}

// Set up voice bearer and TFT
EpsBearer voiceBearer(EpsBearer::GBR_CONV_VOICE);

Ptr<EpcTft> voiceTft = Create<EpcTft>();
EpcTft::PacketFilter dlpfVoice;
dlpfVoice.localPortStart = dlPortVoice;
dlpfVoice.localPortEnd = dlPortVoice;
voiceTft->Add(dlpfVoice);

// Set up voice client and server applications
uint16_t dlPortVoice = 1235;

ApplicationContainer serverApps;
UdpServerHelper dlPacketSinkVoice(dlPortVoice);
serverApps.Add(dlPacketSinkVoice.Install(ueVoiceContainer));

UdpClientHelper dlClientVoice;
dlClientVoice.SetAttribute("RemotePort", UintegerValue(dlPortVoice));
dlClientVoice.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
dlClientVoice.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));
dlClientVoice.SetAttribute("Interval", TimeValue(Seconds(5000.0 / lambdaBe)));

// Create a new EPS bearer for voice traffic with Guaranteed Bit Rate (GBR) and conversational QoS
EpsBearer voiceBearer(EpsBearer::GBR_CONV_VOICE);

// Create a new Traffic Flow Template (TFT) for voice traffic
Ptr<EpcTft> voiceTft = Create<EpcTft>();

// Define a packet filter for downlink voice traffic
EpcTft::PacketFilter dlpfVoice;
// Set the local port range for the downlink voice traffic packet filter
dlpfVoice.localPortStart = dlPortVoice;
dlpfVoice.localPortEnd = dlPortVoice;

// Add the downlink voice traffic packet filter to the voice TFT
voiceTft->Add(dlpfVoice);

// Create a container for client applications
ApplicationContainer clientApps;

// Loop through each UE in the voice container
for (uint32_t i = 0; i < ueVoiceContainer.GetN(); ++i)
{
    // Get the current UE, its net device, and its IP address
    Ptr<Node> ue = ueVoiceContainer.Get(i);
    Ptr<NetDevice> ueDevice = ueVoiceNetDev.Get(i);
    Address ueAddress = ueVoiceIpIface.GetAddress(i);

    // Set the remote address for the downlink client voice application
    dlClientVoice.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    // Install the downlink client voice application on the remote host
    clientApps.Add(dlClientVoice.Install(remoteHost));

    // Activate the dedicated EPS bearer for voice traffic on the UE device
    nrHelper->ActivateDedicatedEpsBearer(ueDevice, voiceBearer, voiceTft);
}

// Start the server and client applications at the specified start time
serverApps.Start(Seconds(udpAppStartTime));
clientApps.Start(Seconds(udpAppStartTime));

// Stop the server and client applications at the specified simulation time
serverApps.Stop(Seconds(simTime));
clientApps.Stop(Seconds(simTime));

// Create a flow monitor helper
FlowMonitorHelper flowmonHelper;

// Create a node container for the endpoint nodes
NodeContainer endpointNodes;
endpointNodes.Add(remoteHost);
endpointNodes.Add(gridScenario.GetUserTerminals());

// Install the flow monitor on the endpoint nodes
Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);

// Set the delay bin width and packet size bin width for the flow monitor
monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

// Create an animation interface
AnimationInterface anim("5G_PF.xml");

// Loop through each UE in the voice container and update the animation
for (uint32_t i = 0; i < ueVoiceContainer.GetN(); ++i)
{
    std::stringstream ss;
    ss << "UE-v" << i + 1;
    anim.UpdateNodeDescription(ueVoiceContainer.Get(i), ss.str());
    anim.UpdateNodeColor(ueVoiceContainer.Get(i), 0, 255, 0);
}

// Update animation for base stations
for (uint32_t i = 0; i < gridScenario.GetBaseStations().GetN(); ++i)
{
    std::stringstream ss;
    ss << "gNB-" << i + 1;
    anim.UpdateNodeDescription(gridScenario.GetBaseStations().Get(i), ss.str());
    anim.UpdateNodeColor(gridScenario.GetBaseStations().Get(i), 255, 0, 0);
}

// Update animation for network nodes
anim.UpdateNodeDescription(pgw, "PGW");
anim.UpdateNodeColor(pgw, 255, 255, 0);

anim.UpdateNodeDescription(sgw, "SGW");
anim.UpdateNodeColor(sgw, 255, 250, 0);

anim.UpdateNodeDescription(mme, "MME");
anim.UpdateNodeColor(mme, 255, 250, 0);

anim.UpdateNodeDescription(remoteHost, "RH");
anim.UpdateNodeColor(remoteHost, 0, 0, 255);

// Enable packet metadata for animation
anim.EnablePacketMetadata(true);

// Stop the simulation at the specified time
Simulator::Stop(Seconds(simTime));

// Run the simulation
Simulator::Run();

// Check for lost packets and retrieve flow statistics
monitor->CheckForLostPackets();
Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

// Initialize variables to track total statistics
double totalRxBytes = 0.0;
double totalDelay = 0.0;
double totalLostPackets = 0.0;
uint32_t totalRxPackets = 0;
uint32_t totalTxPackets = 0;
uint32_t totalFlows = 0;

// Calculate the duration of the flow
double flowDuration = (Seconds(simTime) - Seconds(udpAppStartTime)).GetSeconds();

// Iterate over the flow statistics
for (auto i = stats.begin(); i != stats.end(); ++i)
{
    // Get the 5-tuple for the current flow
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);

    // Print the flow information
    std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
              << t.destinationAddress << ":" << t.destinationPort << ") proto ";
    // Convert protocol number to string
    std::stringstream protoStream;
    protoStream << (uint16_t)t.protocol;
    if (t.protocol == 6)
    {
        protoStream.str("TCP");
    }
    if (t.protocol == 17)
    {
        protoStream.str("UDP");
    }
    std::cout << protoStream.str() << "\n";

    // Print the transmission statistics
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
              << " Mbps\n";

    // Print the reception statistics
    std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";

    // Check if there are received packets
    if (i->second.rxPackets > 0)
    {
        // Calculate the throughput, delay, and loss rate
        double throughput = i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
        double delay = 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
        double lossRate = (i->second.txPackets - i->second.rxPackets) * 100.0 / i->second.txPackets;

        // Print the calculated statistics
        std::cout << "  Throughput: " << throughput << " Mbps\n";
        std::cout << "  Mean delay:  " << delay << " ms\n";
        std::cout << "  Packet loss rate:  " << lossRate << " %\n";

        // Update the total statistics
        totalRxBytes += i->second.rxBytes;
        totalDelay += i->second.delaySum.GetSeconds();
        totalLostPackets += (i->second.txPackets - i->second.rxPackets);
        totalRxPackets += i->second.rxPackets;
        totalTxPackets += i->second.txPackets;
        totalFlows++;
    }
    else
    {
        // Print default values if there are no received packets
        std::cout << "  Throughput:  0 Mbps\n";
        std::cout << "  Mean delay:  0 ms\n";
        std::cout << "  Packet loss rate:  100 %\n";
    }

    // Print the number of received packets
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
}
// Calculate the mean throughput
double meanThroughput = totalRxBytes * 8.0 / (flowDuration * totalFlows) / 1000 / 1000;

// Calculate the mean delay
double meanDelay = totalDelay / totalRxPackets * 1000;

// Calculate the packet loss rate
double packetLossRate = totalLostPackets * 100.0 / totalTxPackets;

// Initialize the fairness index
double fairnessIndex = 0.0;

// Calculate the fairness index if there are multiple flows
if (totalFlows > 1)
{
    double sumThroughput = 0.0;
    double sumThroughputSq = 0.0;
    for (auto i = stats.begin(); i!= stats.end(); ++i)
    {
        // Check if there are received packets for the current flow
        if (i->second.rxPackets > 0)
        {
            // Calculate the throughput for the current flow
            double throughput = i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            sumThroughput += throughput;
            sumThroughputSq += throughput * throughput;
        }
    }
    // Calculate the fairness index using the Jain's fairness index formula
    fairnessIndex = (sumThroughput * sumThroughput) / (totalFlows * sumThroughputSq);
}

// Print the calculated statistics
std::cout << "\n\n  Mean throughput: " << meanThroughput << " Mbps\n";
std::cout << "  Mean delay: " << meanDelay << " ms\n";
std::cout << "  Packet loss rate: " << packetLossRate << " %\n";
std::cout << "  Fairness index: " << fairnessIndex << "\n";

Simulator::Destroy();

std::cout << "Simulation end time: " << Simulator::Now().GetSeconds() << " seconds" << std::endl;

return EXIT_SUCCESS;

}
