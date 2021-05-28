function [children] = singlePointCrossover(parents, children, population_size)
% Takes in parents and produces children using the single point crossover
% method
lower_bound = 2;
upper_bound = 5;
for i = 1:population_size
    parent_one = parents(:,1,i);
    parent_two = parents(:,2,i);
    crossover_point = round(lower_bound + (upper_bound - lower_bound).* rand);
    child = parent_one;
    child(crossover_point:5) = parent_two(crossover_point:5);
    children(:,i) = child;
end

