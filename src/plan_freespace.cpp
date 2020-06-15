#include <intelligent_process_bt/plan_freespace_nodes.h>

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback name="root_Fallback">
                <PlanOMPL/>
                <PlanJointInterp/>
                <PlanTrajOpt/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
 )";

using namespace BT;

int main(int argc, char** argv)
{

    PlanFreespace::PlanRequest request;
    request.start = 0.1;
    request.end = 10.1;

    PlanFreespace::PlanFreespaceManager manager(request);
    BT::BehaviorTreeFactory factory;
    manager.RegisterNodes(factory);

    auto tree = factory.createTreeFromText(xml_text);

    printTreeRecursively(tree.rootNode());

    const bool LOOP = ( argc == 2 && strcmp( argv[1], "loop") == 0);

    do
    {
        NodeStatus status = NodeStatus::RUNNING;
        while( status == NodeStatus::RUNNING)
        {
            status = tree.tickRoot();
            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    }
    while(LOOP);

    PlanFreespace::PlanResult result;
    try
    {
      result = manager.getResult();

      std::cout << "Got traj: ";
      for (auto pt : result.traj)
        std::cout << pt << " ";
      std::cout << std::endl;
    }
    catch (const std::runtime_error& e)
    {
      std::cout << "Failed to get result: " << e.what() << std::endl;
    }

    return 0;
}
