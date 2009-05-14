import commands

class InformationState(object):
    def __init__(self, agent=None):
        self.agent = agent
        self._commands_sent = []
        self._commands_executed = []

    def command_was_sent(self, command):
        self._commands_sent.append(command)

    def command_was_executed(self, command):
        self._commands_executed.append(command)

    def last_command_sent(self, include_passes=False):
        for cmd in reversed(self._commands_sent):
            if include_passes or not isinstance(cmd, commands.PassCommand):
                return cmd

    def last_request_sent(self):
        for cmd in reversed(self._commands_sent):
            if isinstance(cmd, commands.Request):
                return cmd

    def execution_history(self):
        return self._commands_executed
