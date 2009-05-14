import config
import commands

class Scheduler(object):
    """ This class schedules agent activities for MAPSIM. """
    
    def __init__(self, simulator):
        assert len(simulator.agents) >= 1, "scheduler can only be initialized when agents are known"
        self.simulator = simulator
        self.agents = simulator.agents
        self.queue = list(self.agents)
        self.history = []  # structure: [(agt,cmd), ...]
        self.init_sets()

    def init_sets(self):
        self.agents_waiting = set()
        self.agents_done = set()

    def check_for_stop(self, agt, command):
        if self.simulator.time >= config.MAX_TIME_STEPS:
            return commands.StopSimulationCommand(
                "Simulation is still running after %d cycles.  MAPSIM suspects an infinite loop." % 
                config.MAX_TIME_STEPS, success=False)
        if not isinstance(command, commands.PassCommand):
            self.init_sets()
            return False
        group = self.agents_done if command.is_done() else self.agents_waiting
        group.add(agt)
        if self.agents_done == set(self.agents):
            return commands.StopSimulationCommand("All agents have achieved their goals!", success=True)
        not_acting = self.agents_done | self.agents_waiting
        if not_acting == set(self.agents):
            return commands.StopSimulationCommand("No agent is acting anymore!", success=False)
        
    def ask_for_next_command(self):
        agt = self.queue.pop(0)
        agt.log("\nNext agent: %s" % agt, vlevel=2) 
        config.current_agent = agt
#         command = agt.compute_next_action()
        cmds = agt.compute_next_commands()
        command = cmds[0]
        assert str(command.agent).lower() == str(agt).lower(), "%s has agent %s (should be %s)" % (command, command.agent, agt)
        config.current_agent = None
        must_stop = self.check_for_stop(agt, command)
        if must_stop:
            return must_stop
        agt.update_after_command_selection(command)
        return command

    def update_queue(self, agt, command, execution_result):
        import agent 
        assert isinstance(agt, agent.Agent), "%s is of type %s instead of Agent in command %s" % (agt, type(agt), command)
        start, end = 0, len(self.queue)
        ipos = end
        if agt.keep_turn(command, execution_result):
            ipos = start
            agt.log("keeps turn!", vlevel=2)
        # TODO: specific release-turn command
        self.queue.insert(ipos, agt)
        self.history.append((agt, command))

    def execution_result(self, command, execution_result):
        agt = command.agent
        self.update_queue(agt, command, execution_result)
