function [inter_label,next_label,system_liveness] = searchForNextState(automaton,curr_label,alarm,danger)
% check next possible states
state_labels = fieldnames(automaton);
next_state_labels = state_labels(automaton.(curr_label).trans +1);
system_liveness = false;
for i=1:length(next_state_labels)
    temp_label = next_state_labels{i};
    temp_node = automaton.(temp_label);
    temp_a = temp_node.state(1);
    temp_d = temp_node.state(2);
    if alarm == temp_a && danger == temp_d  % if next state match the ENV signal
        if isIntermediateState(temp_node)
            % if next state is an intermediate state
            inter_label = temp_label;
            % check beyond this intermediate state look for next one
            [~,next_label,system_liveness]=searchForNextState(automaton,temp_label,alarm,danger);
        else
            % next state is a state with one system prop
            next_label = temp_label;
            inter_label = curr_label;
            system_liveness = true;
            break;  % next  state is found
        end
    end
end
end