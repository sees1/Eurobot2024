#include <behavior_tree.h>

#include <button_check.h>
#include <push_plan.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "strategy_node");

  try
  {
    int tick_per_in_milliseconds = 1000;
    BT::SequenceNodeWithMemory* root = new BT::SequenceNodeWithMemory("main_sequence");
    BT::ROSCondition* cond = new BT::ROSCondition("button_checker");
    BT::ROSAction* g = new BT::ROSAction("g_action");
    BT::ROSAction* uc0 = new BT::ROSAction("sc_action0");
    BT::ROSAction* uc1 = new BT::ROSAction("sc_action1");
    BT::ROSAction* uc2 = new BT::ROSAction("sc_action2");
    BT::ROSAction* g1 = new BT::ROSAction("g_action1");
    BT::ROSAction* g2 = new BT::ROSAction("g_action2");
    BT::ROSAction* uc3 = new BT::ROSAction("sc_action4");
    BT::ROSAction* uc4 = new BT::ROSAction("sc_action5");

    root->AddChild(cond);
    root->AddChild(g);
    root->AddChild(uc0);
    root->AddChild(uc1);
    root->AddChild(g1);
    root->AddChild(g2);
    root->AddChild(uc2);
    root->AddChild(uc3);
    root->AddChild(uc4);

    Execute(root, tick_per_in_milliseconds);

    delete g;
    delete uc0;
    delete uc1;
    delete g1;
    delete g2;
    delete uc2;
    delete uc3;
    delete uc4;
    delete cond;
    delete root;
  }
  catch (BT::BehaviorTreeException& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}