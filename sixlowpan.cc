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

    Simulator::Schedule(Seconds(duration), [mob, velocityX, velocityY]() {
        std::cout << "[INFO] Invertendo direção do nó móvel!" << std::endl;
        mob->SetVelocity(Vector(-velocityX, -velocityY, 0.0)); // Inverte direção após "duration"
    });
}

int main(int argc, char** argv) {
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
    UpdatePosition(client1, 5.0, 0.0, 100.0, 0.0, 30.0);

    // Mobilidade do Cliente 2.
    MobilityHelper mobility_client2;
    mobility_client2.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility_client2.Install(client2);
    Ptr<ConstantVelocityMobilityModel> mob2 = client2->GetObject<ConstantVelocityMobilityModel>();
    mob2->SetPosition(Vector(0.0, 5.0, 0.0));
    UpdatePosition(client2, 0.0, 5.0, 0.0, 100.0, 30.0);

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
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}