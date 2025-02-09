#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv6-address-generator.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-module.h"

using namespace ns3;

// Atualiza a posição e inverte o movimento.
void UpdatePosition(Ptr<Node> node, double startX, double startY, double endX, double endY, double duration) {
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    double velocityX = (endX - startX) / duration;
    double velocityY = (endY - startY) / duration;
    mob->SetVelocity(Vector(velocityX, velocityY, 0.0));

    // Quando atingir o destino, o nó PARA.
    Simulator::Schedule(Seconds(duration), [mob]() {
        std::cout << "[INFO] Nó chegou ao destino e parou!" << std::endl;
        mob->SetVelocity(Vector(0.0, 0.0, 0.0)); // Faz o nó PARAR no destino.
    });
}

void CollectSignalData(Ptr<Node> node, Ptr<Node> ap, Ptr<PropagationLossModel> lossModel) {
    double distance = node->GetObject<MobilityModel>()->GetDistanceFrom(ap->GetObject<MobilityModel>());

    // Potência do transmissor (Tx Power) para 802.15.4 (pode variar entre -10 a 0 dBm)
    double txPower = 0.0;

    // Calculando potência do sinal recebido (Rx Power)
    double rxPower = lossModel->CalcRxPower(txPower, node->GetObject<MobilityModel>(), ap->GetObject<MobilityModel>());

    // Definição do ruído térmico para 802.15.4 (típico: -101 dBm)
    double noisePower = -101.0;

    // Calculando SNR corretamente
    double snr = rxPower - noisePower;

    std::cout << "[Sinal] Tempo: " << Simulator::Now().GetSeconds()
              << "s | Distância: " << distance
              << "m | Rx Power: " << rxPower << " dBm | SNR: " << snr << " dB" << std::endl;

    // Agendar próxima coleta de dados (enquanto a simulação estiver rodando)
    if (Simulator::Now().GetSeconds() < 60) {
        Simulator::Schedule(Seconds(1.0), &CollectSignalData, node, ap, lossModel);
    }
}

int main(int argc, char** argv) {

    bool verbose = false;
    bool disablePcap = false;
    bool disableAsciiTrace = false;
    bool enableLSixlowLogLevelInfo = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "turn on log components", verbose);
    cmd.AddValue("disable-pcap", "disable PCAP generation", disablePcap);
    cmd.AddValue("disable-asciitrace", "disable ascii trace generation", disableAsciiTrace);
    cmd.AddValue("enable-sixlowpan-loginfo",
                 "enable sixlowpan LOG_LEVEL_INFO (used for tests)",
                 enableLSixlowLogLevelInfo);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("Ping", LOG_LEVEL_ALL);
        LogComponentEnable("LrWpanMac", LOG_LEVEL_ALL);
        LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LrWpanNetDevice", LOG_LEVEL_ALL);
        LogComponentEnable("SixLowPanNetDevice", LOG_LEVEL_ALL);
    }

    // Crio 3 nodes e ponteiros para mais controle sobre cada nó individualmente.
    Ptr<Node> ap = CreateObject<Node>();
    Ptr<Node> client1 = CreateObject<Node>();
    Ptr<Node> client2 = CreateObject<Node>();
    NodeContainer nodes(ap, client1, client2);

    // Defino a mobilidade do AP.
    MobilityHelper mobility_ap;
    mobility_ap.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility_ap.Install(ap);

    // Definir explicitamente a posição da AP em (0, 0, 0)
    Ptr<MobilityModel> mobility = ap->GetObject<MobilityModel>();
    mobility->SetPosition(Vector(0.0, 0.0, 0.0));

    // Mobilidade do Cliente 1.
    MobilityHelper mobility_client1;
    mobility_client1.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility_client1.Install(client1);
    Ptr<ConstantVelocityMobilityModel> mob1 = client1->GetObject<ConstantVelocityMobilityModel>();
    mob1->SetPosition(Vector(5.0, 0.0, 0.0));
    UpdatePosition(client1, 5.0, 0.0, 100.0, 0.0, 58.0);

    // Mobilidade do Cliente 2.
    MobilityHelper mobility_client2;
    mobility_client2.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility_client2.Install(client2);
    Ptr<ConstantVelocityMobilityModel> mob2 = client2->GetObject<ConstantVelocityMobilityModel>();
    mob2->SetPosition(Vector(0.0, 5.0, 0.0));
    UpdatePosition(client2, 0.0, 5.0, 0.0, 100.0, 58.0);

    // Criar modelo de propagação manualmente.
    Ptr<LogDistancePropagationLossModel> logDistanceModel = CreateObject<LogDistancePropagationLossModel>();
    logDistanceModel->SetAttribute("Exponent", DoubleValue(4.5));

    // Criar modelo de Nakagami manualmente.
    Ptr<NakagamiPropagationLossModel> nakagamiModel = CreateObject<NakagamiPropagationLossModel>();
    nakagamiModel->SetAttribute("m0", DoubleValue(0.8));
    nakagamiModel->SetAttribute("m1", DoubleValue(0.5));
    nakagamiModel->SetAttribute("m2", DoubleValue(0.2));

    // Conectar modelos de propagação.
    logDistanceModel->SetNext(nakagamiModel);

    // Adicionar modelos de propagação ao helper.
    LrWpanHelper lrWpanHelper;
    lrWpanHelper.AddPropagationLossModel("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(4.5));
    lrWpanHelper.AddPropagationLossModel("ns3::NakagamiPropagationLossModel", "m0", DoubleValue(0.8), "m1", DoubleValue(0.5), "m2", DoubleValue(0.2));

    // Instalar os dispositivos LR-WPAN nos nós.
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);

    // PAN association manual.
    lrWpanHelper.CreateAssociatedPan(lrwpanDevices, 0);

    // Pilha IPv6.
    InternetStackHelper internetv6;
    internetv6.Install(nodes);

    // Sixlowpan.
    SixLowPanHelper sixlowpan;
    NetDeviceContainer devices = sixlowpan.Install(lrwpanDevices);

    // Aplicando IPv6 aos dispositivos.
    Ipv6AddressHelper ipv6;
    ipv6.SetBase(Ipv6Address("2001:2::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer deviceInterfaces;
    deviceInterfaces = ipv6.Assign(devices);

    for (uint32_t i = 0; i < devices.GetN(); i++) {
        std::cout << "Device " << i << ": pseudo-Mac-48 "
                  << Mac48Address::ConvertFrom(devices.Get(i)->GetAddress())
                  << ", IPv6 Address " << deviceInterfaces.GetAddress(i, 1)
                  << std::endl;
    }
    std::cout << "Total de dispositivos: " << devices.GetN() << std::endl;

    // Pings
    uint32_t packetSize = 32; // Tamanho dos pacotes.
    uint32_t maxPacketCount = 100; // Quantidade de pacotes.
    Time interPacketInterval = Seconds(0.1);

    // Enviar pacotes para o Nó 1
    PingHelper ping1(deviceInterfaces.GetAddress(1, 1));
    ping1.SetAttribute("Count", UintegerValue(maxPacketCount));
    ping1.SetAttribute("Interval", TimeValue(interPacketInterval));
    ping1.SetAttribute("Size", UintegerValue(packetSize));
    ApplicationContainer apps1 = ping1.Install(nodes.Get(0));

    // Enviar pacotes para o Nó 2
    PingHelper ping2(deviceInterfaces.GetAddress(2, 1));  // Agora para o Nó 2
    ping2.SetAttribute("Count", UintegerValue(maxPacketCount));
    ping2.SetAttribute("Interval", TimeValue(interPacketInterval));
    ping2.SetAttribute("Size", UintegerValue(packetSize));
    ApplicationContainer apps2 = ping2.Install(nodes.Get(0));

    // Iniciar e parar os aplicativos no mesmo período
    apps1.Start(Seconds(2));
    apps1.Stop(Seconds(60));

    apps2.Start(Seconds(2));
    apps2.Stop(Seconds(60));

    if (!disableAsciiTrace) {
        AsciiTraceHelper ascii;
        lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Ping-6LoW-lr-wpan.tr"));
    }

    if (!disablePcap) {
        lrWpanHelper.EnablePcapAll(std::string("Ping-6LoW-lr-wpan"), true);
    }

    if (enableLSixlowLogLevelInfo) {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
        Ipv6RoutingHelper::PrintNeighborCacheAllAt(Seconds(9), routingStream);
    }

    // Configuração do NetAnim.
    AnimationInterface anim("sixlowpan-animation.xml");
    anim.SetMobilityPollInterval(Seconds(0.1));
    anim.UpdateNodeDescription(ap, "AP");
    anim.UpdateNodeDescription(client1, "Jarbas");
    anim.UpdateNodeDescription(client2, "Grazy");
    anim.UpdateNodeColor(ap, 255, 0, 0);
    anim.UpdateNodeColor(client1, 0, 255, 0);
    anim.UpdateNodeColor(client2, 0, 0, 255);

    // Configuração do Simulador.
    Simulator::Stop(Seconds(60));

    // Iniciar coleta de sinal para ambos os clientes.
    Simulator::Schedule(Seconds(1.0), &CollectSignalData, client1, ap, logDistanceModel);
    Simulator::Schedule(Seconds(1.0), &CollectSignalData, client2, ap, logDistanceModel);

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}