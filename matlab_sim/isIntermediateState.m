function bool = isIntermediateState(node)

system_prop = find(node.state(3:8)')-1;

bool = length(system_prop) ==2;
end