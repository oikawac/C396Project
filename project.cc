#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"

#define YELLOW_CODE "\033[33m"
#define TEAL_CODE "\033[36m"
#define BOLD_CODE "\033[1m"
#define END_CODE "\033[0m"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WaveProject");

int unique_req_id = 0;
int unique_rep_id = 0;

/*
    Trace functions Rx, RxDrop and WAVE channel setup taken from:

    ns3 Shared Resource by Adil Alsuhaim - https://github.com/addola/NS3-HelperScripts
*/
std::stringstream global_debug_rx;
void Rx (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector,MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t extra)
{
	//context will include info about the source of this event. Use string manipulation if you want to extract info.
	global_debug_rx << BOLD_CODE <<  context << END_CODE << std::endl;
	//Print the info.
	global_debug_rx << "\tSize=" << packet->GetSize()
			  << " Freq="<<channelFreqMhz
			  << " Mode=" << txVector.GetMode()
			  << " Signal=" << signalNoise.signal
			  << " Noise=" << signalNoise.noise << std::endl;

	//We can also examine the WifiMacHeader
	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		global_debug_rx << "\tDestination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2() << std::endl;
	}
}
std::stringstream global_debug_rxdrop;
void RxDrop (std::string context, Ptr<const Packet> packet, ns3::WifiPhyRxfailureReason reason)
{
	global_debug_rxdrop << BOLD_CODE << YELLOW_CODE << "Packet Rx Dropped!" << END_CODE << std::endl;
	//From ns-3.30, the reasons are defined in an enum type in ns3::WifiPhy class.
	global_debug_rxdrop << " Reason : " << reason << std::endl;
	global_debug_rxdrop << context << std::endl;

	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		global_debug_rxdrop << " Destination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2() << "\tSeq No. " << hdr.GetSequenceNumber() << std::endl;
	}
}

struct Rect {
    double x;
    double y;
    double w;
    double h;
    bool contains(int px, int py) {
        return (px > x && px <= x + w && py > y && py <= y + h);
    }
    bool contains(Vector p) {
        return (p.x > x && p.x <= x + w && p.y > y && p.y <= y + h);
    }
};

struct Datafield {
    int tile_count;
    int tile_width;
    double min_x;
    double min_y;
    int x_tile_limit;
    int y_tile_limit;
    Rect *tile_array;//tiles
    double *data_array;//data measured at tile
    Time *time_collected_array;//age of information
    int *decision_event_array;//1 if tx should go ahead 0 if tx cancelled
};

//datafield which exists to track all tile measurements so that we can compare the local knowledge of mobile nodes against best possible knowledge available
Datafield df_global;

class DatafieldApp : public Application {
public: 
    static TypeId GetTypeId () {
        static TypeId tid = TypeId("ns3::DatafieldApp")
                .SetParent <Application> ()
                .AddConstructor<DatafieldApp> ()
                ;
        return tid;
    }
    virtual TypeId GetInstanceTypeId () const {
        return GetTypeId();
    }
    DatafieldApp() {
        m_running = false;
        m_mode = WifiMode("OfdmRate6MbpsBW10MHz");
    }
    ~DatafieldApp() { }
    void SetWifiMode (WifiMode mode) {
        m_mode = mode;
    }
    bool ReceivePacket (Ptr<NetDevice> device,Ptr<const Packet> packet,uint16_t protocol, const Address &sender) {
        if (!m_running) return false; 
        uint8_t *buffer = new uint8_t[packet->GetSize()];
        packet->CopyData(buffer, packet->GetSize());
        int bytesRemaining = packet->GetSize();
        int index = 0;
        int req_id = -1;
        memcpy(&req_id, &buffer[index], sizeof(int));
        index += sizeof(int);
        int req_rep_indicator = -1;
        memcpy(&req_rep_indicator, &buffer[index], sizeof(int));
        index += sizeof(int);
        if (req_rep_indicator == 0) {
            m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" received request for "<<(int)((bytesRemaining-sizeof(int))/(sizeof(int)+sizeof(double)))<<" reqid["<<req_id<<"]\n";
            while (index < bytesRemaining) {
                int tile_id;
                memcpy(&tile_id, &buffer[index], sizeof(int));
                index += sizeof(int);
                double aoi_in_seconds;
                memcpy(&aoi_in_seconds, &buffer[index], sizeof(double));
                index += sizeof(double);
                Time request_aoi = Seconds(aoi_in_seconds);
                Time local_aoi = Now()-m_datafield.time_collected_array[tile_id];
                m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" considering request for tile"<<tile_id<<" loc_aoi: "<<local_aoi.GetSeconds()<<" req_aoi: "<<request_aoi.GetSeconds()<<" reqid["<<req_id<<"]\n";
                if (local_aoi < request_aoi && m_datafield.decision_event_array[tile_id] == 0) {
                    Time delay = Seconds(local_aoi.GetSeconds()/request_aoi.GetSeconds());
                    Simulator::Schedule(delay, &DatafieldApp::DecisionOnTx, this, tile_id, req_id);
                    m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" will decide on tx tile"<<tile_id<<" in linear delay: "<<delay.GetSeconds()<<" reqid["<<req_id<<"]\n";
                    m_datafield.decision_event_array[tile_id] = 1;
                }
            }
        } else if (req_rep_indicator == 1) {
            while (index < bytesRemaining) {
                int rep_id;
                memcpy(&rep_id, &buffer[index], sizeof(int));
                index += sizeof(int);
                int tile_id;
                memcpy(&tile_id, &buffer[index], sizeof(int));
                index += sizeof(int);
                double data;
                memcpy(&data, &buffer[index], sizeof(double));
                index += sizeof(double);
                double aoi_in_seconds;
                memcpy(&aoi_in_seconds, &buffer[index], sizeof(double));
                index += sizeof(double);
                Time reply_aoi = Seconds(aoi_in_seconds);
                if (Now()-m_datafield.time_collected_array[tile_id] > reply_aoi) {
                    m_datafield.data_array[tile_id] = data;
                    m_datafield.time_collected_array[tile_id] = Now()-reply_aoi;
                    m_datafield.decision_event_array[tile_id] = 0;//cancel a pending tx decision on this tile
                    m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" collecting tile"<<tile_id<<" rep_aoi "<<reply_aoi.GetSeconds()<<" reqid["<<req_id<<"] repid["<<unique_rep_id<<"]\n";
                }
            }
        } else {
            m_local_debug_history<<" "<<Now().GetSeconds()<<" received unknown "<<m_label<<" "<<req_rep_indicator<<" reqid["<<req_id<<"]\n";
        }
        delete[] buffer;
        return true; 
    }
    void RequestNearby(int radius, Time stale_threshold, Time interval) {
        if (!m_running) return;
        int count = 0;
        for (int i=0; i<m_datafield.tile_count; i++) {
            if (Now()-m_datafield.time_collected_array[i] > stale_threshold) {
                Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
                Vector position = mob->GetPosition();
                double x_diff = (position.x-(m_datafield.tile_array[i].x+m_datafield.tile_array[i].w/2));
                double y_diff = (position.y-(m_datafield.tile_array[i].y+m_datafield.tile_array[i].h/2));
                if (radius == 0) count++;
                else if (x_diff*x_diff+y_diff*y_diff < radius*radius) {
                    count++;
                }
            }
        }
        int max_size = (4000-2*sizeof(int))/(sizeof(int)+sizeof(double));
        int last_tile = 0;
        while (count > 0) {
            int packet_count = count;
            if (count > max_size) {
                packet_count = max_size;
            }
            count -= packet_count;
            char *buffer = new char[2*sizeof(int)+packet_count*(sizeof(int)+sizeof(double))];
            unique_req_id++;
            memcpy(&buffer[0], &unique_req_id, sizeof(int));
            m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" requests "<<packet_count<<" tiles reqid["<<unique_req_id<<"]\n";
            int zero = 0;
            memcpy(&buffer[sizeof(int)], &zero, sizeof(int));
            int byte_index = 2*sizeof(int);
            int tiles_added = 0;
            for (int i=last_tile; i<m_datafield.tile_count; i++) {
                last_tile = i+1;
                if (Now()-m_datafield.time_collected_array[i] > stale_threshold) {
                    Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
                    Vector position = mob->GetPosition();
                    double x_diff = (position.x-(m_datafield.tile_array[i].x+m_datafield.tile_array[i].w/2));
                    double y_diff = (position.y-(m_datafield.tile_array[i].y+m_datafield.tile_array[i].h/2));
                    if (radius == 0) {
                        tiles_added++;
                        double loc_aoi = (Now()-m_datafield.time_collected_array[i]).GetSeconds();
                        memcpy(&buffer[byte_index], &i, sizeof(int));
                        byte_index += sizeof(int);
                        memcpy(&buffer[byte_index], &loc_aoi, sizeof(double));
                        byte_index += sizeof(double);
                    } else if (x_diff*x_diff+y_diff*y_diff < radius*radius) {
                        tiles_added++;
                        double loc_aoi = (Now()-m_datafield.time_collected_array[i]).GetSeconds();
                        memcpy(&buffer[byte_index], &i, sizeof(int));
                        byte_index += sizeof(int);
                        memcpy(&buffer[byte_index], &loc_aoi, sizeof(double));
                        byte_index += sizeof(double);
                    }
                }
                if (tiles_added == packet_count) break;
            }
            //NS_LOG_INFO("SIZE "<<sizeof(int)+packet_count*(sizeof(int)+sizeof(double)));
            Ptr<Packet> packet = Create<Packet> ((uint8_t*)buffer, 2*sizeof(int)+packet_count*(sizeof(int)+sizeof(double)));
            Simulator::Schedule(Seconds(0), &DatafieldApp::BroadcastPacket, this, packet);
            delete[] buffer;
        }
        Simulator::Schedule(interval, &DatafieldApp::RequestNearby, this, radius, stale_threshold, interval);
    }
    void DecisionOnTx(int tile, int req_id) {
        if (m_datafield.decision_event_array[tile] == 1) {
            char *buffer = new char[3*sizeof(int)+(sizeof(int)+2*sizeof(double))];
            int byte_index = 0;
            memcpy(&buffer[byte_index], &req_id, sizeof(int));
            byte_index += sizeof(int);
            int one = 1;
            memcpy(&buffer[byte_index], &one, sizeof(int));
            byte_index += sizeof(int);
            unique_rep_id++;
            memcpy(&buffer[byte_index], &unique_rep_id, sizeof(int));
            byte_index += sizeof(int);
            m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" providing tile"<<tile<<" reqid["<<req_id<<"] repid["<<unique_rep_id<<"]\n";
            double data = m_datafield.data_array[tile];
            double aoi = (Now()-m_datafield.time_collected_array[tile]).GetSeconds();
            memcpy(&buffer[byte_index], &tile, sizeof(int));
            byte_index += sizeof(int);
            memcpy(&buffer[byte_index], &data, sizeof(double));
            byte_index += sizeof(double);
            memcpy(&buffer[byte_index], &aoi, sizeof(double));
            byte_index += sizeof(double);
            Ptr<Packet> packet = Create<Packet> ((uint8_t*)buffer, 3*sizeof(int)+(sizeof(int)+2*sizeof(double)));
            delete[] buffer;
            m_datafield.decision_event_array[tile] = 0;
            Simulator::Schedule(Seconds(0), &DatafieldApp::BroadcastPacket, this, packet);
        } else {
            m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" "<<" cancels tx for tile"<<tile<<" reqid["<<req_id<<"]\n";
        }
    }
    void BroadcastPacket(Ptr<Packet> packet) {
        m_bytes_transmitted += packet->GetSize();
        TxInfo tx;
        tx.preamble = WIFI_PREAMBLE_LONG;
        tx.channelNumber = CCH; 
        tx.priority = 7; //highest priority.
        tx.txPowerLevel = 1; //default power level is maximum
        tx.dataRate = m_mode;
        m_waveDevice->SendX(packet, Mac48Address::GetBroadcast(), 0x88dc, tx);
    }
    void SetDatafield(Datafield df) {
        m_datafield = df;
    }
    void MeasureData(Time interval) {
        if (m_running) {
            Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
            Vector position = mob->GetPosition();
            for (int i=0; i<m_datafield.tile_count; i++) {
                if (m_datafield.tile_array[i].contains(position)) {
                    m_datafield.data_array[i] = 1;
                    m_datafield.time_collected_array[i] = Now();
                    df_global.data_array[i] = 1;
                    df_global.time_collected_array[i] = Now();
                    m_local_debug_history<<" time"<<Now().GetSeconds()<<" node"<<m_label<<" "<<" measures tile"<<i<<"\n";
                }
            }
            Simulator::Schedule(interval, &DatafieldApp::MeasureData, this, interval);
        }
    }
    void LogAoiStatistics(int num_radius_intervals, int delta_radius) {
        if (!m_running) return;
        double* radii_aoi_sum = new double[num_radius_intervals] { 0 };
        int* radii_tile_count = new int[num_radius_intervals] { 0 };
        double* g_radii_aoi_sum = new double[num_radius_intervals] { 0 };
        int* g_radii_tile_count = new int[num_radius_intervals] { 0 };
        for (int i=0; i<m_datafield.tile_count; i++) {//calculate local and global aoi within specified radii
            Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
            Vector position = mob->GetPosition();
            double x_diff = (position.x-(m_datafield.tile_array[i].x+m_datafield.tile_array[i].w/2));
            double y_diff = (position.y-(m_datafield.tile_array[i].y+m_datafield.tile_array[i].h/2));
            double dist = sqrt(x_diff*x_diff+y_diff*y_diff);
            int radii_delta_discrete = (int)(floor(dist/delta_radius));
            for (int radii_index=0; radii_index<num_radius_intervals; radii_index++) {
                if (radii_delta_discrete <= radii_index) {
                    radii_aoi_sum[radii_index] += (Now()-m_datafield.time_collected_array[i]).GetSeconds();
                    radii_tile_count[radii_index]++;
                    g_radii_aoi_sum[radii_index] += (Now()-df_global.time_collected_array[i]).GetSeconds();
                    g_radii_tile_count[radii_index]++;
                }   
            }
        }
        for (int i=0; i<num_radius_intervals; i++) {
            m_local_debug_aoi_stats << " time"<<(Now().GetSeconds())
                                    <<" node"<<m_label<<" avg. aoi of tiles within radius"<<((i+1)*delta_radius)<<"   "<<(radii_aoi_sum[i]/radii_tile_count[i])
                                    <<"   best existing "<<(g_radii_aoi_sum[i]/g_radii_tile_count[i])<<"  ["<<(radii_aoi_sum[i]/radii_tile_count[i])/(g_radii_aoi_sum[i]/g_radii_tile_count[i])
                                    <<"]\n";
        }
        delete [] radii_aoi_sum;
        delete [] radii_tile_count;
    }
    void DebugOutputAoiStatistics() {
        std::ofstream output_file("aoi_stats_output_"+std::to_string(m_label)+".txt");
        if (!output_file.is_open()) {
            std::cerr << "Could not open the file" << std::endl;
            exit(1);
        }
        output_file << m_local_debug_aoi_stats.rdbuf();
        output_file.close();
    }
    void DebugOutputDatafield() {
        if (!m_running) return;
        std::ofstream output_file("datafield_output_"+std::to_string(m_label)+"["+std::to_string(Now().GetSeconds())+"].txt");
        if (!output_file.is_open()) {
            std::cerr << "Could not open the file" << std::endl;
            exit(1);
        }
        for (int i=0; i<m_datafield.tile_count; i++) {
            output_file << m_datafield.data_array[i]<<" ";
            output_file << (Now()-m_datafield.time_collected_array[i]).GetSeconds()<<" ";
        }
        output_file.close();
    }
    void DebugOutputHistory() {
        std::ofstream output_file("history_output_"+std::to_string(m_label)+".txt");
        if (!output_file.is_open()) {
            std::cerr << "Could not open the file" << std::endl;
            exit(1);
        }
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector position = mob->GetPosition();
        output_file << "total bytes transmitted: " << m_bytes_transmitted<<"\n";
        output_file << "final position: " << position.x <<" "<<position.y<<"\n";
        output_file << m_local_debug_history.rdbuf();
        output_file.close();
    }
    int m_label;
    bool m_running;
private:
    void StartApplication() {
        m_bytes_transmitted = 0;
        m_local_debug_history<<" time"<<Now().GetSeconds()<<" application starts\n";
        m_running = true;
        Ptr<Node> node = GetNode();
        Ptr<NetDevice> device = node->GetDevice(0);
        m_waveDevice = DynamicCast<WaveNetDevice>(device);
        device->SetReceiveCallback(MakeCallback (&DatafieldApp::ReceivePacket, this));
    }
    void StopApplication() {
        m_local_debug_history<<" time"<<Now().GetSeconds()<<" application stops\n";
        m_running = false;
        DebugOutputHistory();
        DebugOutputAoiStatistics();
    }
    Ptr<WaveNetDevice> m_waveDevice;
    WifiMode m_mode;
    Datafield m_datafield;
    std::stringstream m_local_debug_history;
    std::stringstream m_local_debug_aoi_stats;
    int m_bytes_transmitted;
};

void log_sim_progress() {
    NS_LOG_INFO(" "<<Now().GetSeconds());
    Simulator::Schedule(Seconds(1), &log_sim_progress);
}

void debug_output_global_datafield(int tile_count) {
    std::ofstream output_file("datafield_output_99990000["+std::to_string(Now().GetSeconds())+"].txt");
    if (!output_file.is_open()) {
        std::cerr << "Could not open the file" << std::endl;
        exit(1);
    }
    for (int i=0; i<tile_count; i++) {
        output_file << df_global.data_array[i]<<" ";
        output_file << (Now()-df_global.time_collected_array[i]).GetSeconds()<<" ";
    }
    output_file.close();
}

int main(int argc, char *argv[])
{
    std::string traceFile;
    std::string nodeFile;
    std::string tileFile;
    std::string ifFile;

    int param_p;
    int param_a = 300;
    int param_r;
    int nodeCount;
    int ifCount;
    double duration;

    LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_INFO);
    
    CommandLine cmd;
    cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
    cmd.AddValue ("nodeFile", "Node data file", nodeFile);
    cmd.AddValue ("tileFile", "Tile data file", tileFile);
    cmd.AddValue ("ifLocFile", "Infrastructure locations file", ifFile);
    cmd.AddValue ("ifCount", "Infrastructure node count", ifCount);
    cmd.AddValue ("nodeCount", "Number of nodes", nodeCount);
    cmd.AddValue ("duration", "Duration of Simulation", duration);
    cmd.AddValue("param_r", "param_r", param_r);
    cmd.AddValue("param_p", "param_p", param_p);
    cmd.Parse(argc, argv);

    if (traceFile.empty() || nodeCount <= 0 || duration <= 0 || nodeFile.empty() || tileFile.empty())
    {
      return 1;
    }

    LogComponentEnable("WaveProject", LOG_INFO);

    Ns2MobilityHelper ns2_mobility = Ns2MobilityHelper (traceFile);

    NodeContainer nodes;
    nodes.Create(nodeCount);

    //Add SUMO generated traces to nodes
    ns2_mobility.Install();

    NodeContainer if_nodes;
    if (ifCount > 0) {
        //IF mobility
        MobilityHelper mobility;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

        if_nodes.Create(ifCount);
        std::ifstream if_data(ifFile);
        if (!if_data.is_open()) {
            std::cerr << "Could not open the file - '"
                << ifFile << "'" << std::endl;
            exit(1);
        }
        double x;
        double y;
        while (if_data >> x) {
            if_data >> y;
            NS_LOG_UNCOND("if loc: "<<x<<" "<<y);
            positionAlloc->Add (Vector (x, y, 0.0));
        }
        if_data.close();
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(if_nodes);
    }
    //Wireless Acess in Vehicular Environment Physical Channel
    YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
    YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default();
    wavePhy.SetChannel(waveChannel.Create());
    wavePhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    //Power Levels - 33 dBm is the highest available specified by 802.11
    wavePhy.Set("TxPowerStart", DoubleValue(33));
    wavePhy.Set("TxPowerEnd", DoubleValue (33));

    //MAC Layer
    QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
    WaveHelper waveHelper = WaveHelper::Default();

    waveHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager", 
        "DataMode", StringValue("OfdmRate6MbpsBW10MHz"), 
        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"),
        "NonUnicastMode", StringValue("OfdmRate6MbpsBW10MHz"));
    
    NetDeviceContainer devices = waveHelper.Install(wavePhy, waveMac, nodes);
    NetDeviceContainer devices_if;
    if (ifCount > 0) {
        devices_if = waveHelper.Install(wavePhy, waveMac, if_nodes);
    }
    wavePhy.EnablePcap("WaveProject", devices, true);

    //parse node data
    double *starting_times = new double[nodeCount];
    for (int i=0; i<nodeCount; i++) starting_times[i] = -1.0;
    double *finishing_times = new double[nodeCount];
    std::ifstream node_data(nodeFile);
    if (!node_data.is_open()) {
        std::cerr << "Could not open the file - '"
             << nodeFile << "'" << std::endl;
        exit(1);
    }
    double time;
    while (node_data >> time) {
        int node;
        node_data >> node;
        if (starting_times[node] < 0) starting_times[node] = time;
        finishing_times[node] = time;
        double x;
        node_data >> x;
        double y;
        node_data >> y;
    }
    node_data.close();

    //parse tile data
    std::ifstream tile_data(tileFile);
    if (!tile_data.is_open()) {
        std::cerr << "Could not open the file - '"
             << tileFile << "'" << std::endl;
        exit(1);
    }
    int tile_count;
    tile_data >> tile_count;
    double min_x;
    tile_data >> min_x;
    double min_y;
    tile_data >> min_y;
    int tile_width;
    tile_data >> tile_width;
    int x_limit;
    tile_data >> x_limit;
    int y_limit;
    tile_data >> y_limit;
    int tile_x;
    int tile_y;
    Rect *tile_array = new Rect[tile_count];
    int index = 0;
    while (tile_data >> tile_x) {
        tile_data >> tile_y;
        tile_array[index].x = tile_x*15+min_x;
        tile_array[index].y = tile_y*15+min_y;
        tile_array[index].w = tile_width;
        tile_array[index].h = tile_width;
        index++;
    }
    tile_data.close();

    //init applications
    for (int i=0; i<nodeCount; i++) {
        Ptr<DatafieldApp> app = Create<DatafieldApp>();
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(starting_times[i]));
        app->SetStopTime(Seconds(finishing_times[i]));
        app->m_label = i;
        Datafield df;
        df.tile_count = tile_count;
        df.tile_width = tile_width;
        df.min_x = min_x;
        df.min_y = min_y;
        df.x_tile_limit = x_limit;
        df.y_tile_limit = y_limit;
        df.tile_array = tile_array;//shared pointer since all the same and not mutated
        df.data_array = new double[tile_count]{ 0 };
        df.time_collected_array = new Time[tile_count];
        for (int j=0; j<tile_count; j++) {
            df.time_collected_array[j] = Seconds(0);
        }
        df.decision_event_array = new int[tile_count]{ 0 };
        app->SetDatafield(df);
        Time interval = Seconds(1);
        Simulator::Schedule(Seconds(starting_times[i])+interval, &DatafieldApp::MeasureData, app, interval);
        Simulator::Schedule(Seconds(starting_times[i]+1), &DatafieldApp::RequestNearby, app, param_r, Seconds(param_a), Seconds(param_p));
        for (float j=starting_times[i]; j+10.0<finishing_times[i]; j += 10.0) {
            Simulator::Schedule(Seconds(j), &DatafieldApp::DebugOutputDatafield, app);  
        }
        for (float j=starting_times[i]; j+2.5<finishing_times[i]; j += 2.5) {
            Simulator::Schedule(Seconds(j), &DatafieldApp::LogAoiStatistics, app, 10, 30);
        }
    }
    df_global.tile_count = tile_count;
    df_global.tile_width = tile_width;
    df_global.min_x = min_x;
    df_global.min_y = min_y;
    df_global.x_tile_limit = x_limit;
    df_global.y_tile_limit = y_limit;
    df_global.tile_array = tile_array;//shared pointer since all the same and not mutated
    df_global.data_array = new double[tile_count]{ 0 };
    df_global.time_collected_array = new Time[tile_count];
    for (int j=0; j<tile_count; j++) {
        df_global.time_collected_array[j] = Seconds(0);
    }
    for (float j=2; j+10.0<duration-1; j += 10.0) {
        Simulator::Schedule(Seconds(j), &debug_output_global_datafield, tile_count);  
    }
    //datafield shared by infrastructure access points
    Datafield df_if;
    if (ifCount > 0) {
        df_if.tile_count = tile_count;
        df_if.tile_width = tile_width;
        df_if.min_x = min_x;
        df_if.min_y = min_y;
        df_if.x_tile_limit = x_limit;
        df_if.y_tile_limit = y_limit;
        df_if.tile_array = tile_array;//shared pointer since all the same and not mutated
        df_if.data_array = new double[tile_count]{ 0 };
        df_if.time_collected_array = new Time[tile_count];
        for (int j=0; j<tile_count; j++) {
            df_if.time_collected_array[j] = Seconds(0);
        }
        df_if.decision_event_array = new int[tile_count]{ 0 };
        for (int i=0; i<ifCount; i++) {
            Ptr<DatafieldApp> app = Create<DatafieldApp>();
            if_nodes.Get(i)->AddApplication(app);
            app->SetStartTime(Seconds(1));
            app->SetStopTime(Seconds(duration-1));
            app->m_label = 77770000+i;
            app->SetDatafield(df_if);
            Simulator::Schedule(Seconds(2), &DatafieldApp::RequestNearby, app, 0, Seconds(param_a), Seconds(param_p));
            for (float j=2; j+10.0<duration-1; j += 10.0) {
                Simulator::Schedule(Seconds(j), &DatafieldApp::DebugOutputDatafield, app);
            }
            for (float j=2; j+2.5<duration-1; j += 2.5) {
                Simulator::Schedule(Seconds(j), &DatafieldApp::LogAoiStatistics, app, 10, 30);
            }
        }
    }

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/MonitorSnifferRx", MakeCallback(&Rx));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/PhyRxDrop", MakeCallback(&RxDrop));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/TxPowerLevels", ns3::UintegerValue(1));
    Simulator::Schedule(Seconds(1), &log_sim_progress);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    Simulator::Destroy();

    std::ofstream output_file_rx("debug_output_rx.txt");
    if (!output_file_rx.is_open()) {
        std::cerr << "Could not open the file" << std::endl;
        exit(1);
    }
    output_file_rx << global_debug_rx.rdbuf();
    output_file_rx.close();
    std::ofstream output_file_rxdrop("debug_output_rxdrop.txt");
    if (!output_file_rxdrop.is_open()) {
        std::cerr << "Could not open the file" << std::endl;
        exit(1);
    }
    output_file_rxdrop << global_debug_rxdrop.rdbuf();
    output_file_rxdrop.close();

    return 0;
}