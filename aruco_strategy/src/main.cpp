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
    BT::ROSCondition* cond   = new BT::ROSCondition("button_checker");
    BT::ROSAction*    mb1 = new BT::ROSAction("mb1_action");
    // BT::ROSAction*    mb2 = new BT::ROSAction("mb2_action");

    root->AddChild(cond);
    root->AddChild(mb1);
    // root->AddChild(mb2);

    Execute(root, tick_per_in_milliseconds);

    delete mb1;
    // delete mb2;
    delete cond;
    delete root;
  }
  catch(BT::BehaviorTreeException& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}