function [parents] = selection_truncation(Jvec, population, population_size, k, parents)
% Takes in fitness evaluations, population, population size,
% and cutoff (k) and returns parents of the population

% First choose best k chromosomes
top_performers = zeros(5, k);
for i = 1:k
    [m indice] = min(Jvec);
    top_performers(:,i) = population(:,indice);
    Jvec(indice) = inf;
end

% Generate parents from the best k chromosomes
lower_bound = 1;
upper_bound = k;
for j = 1:1:population_size
    parent_indexes = round(lower_bound + (upper_bound - lower_bound).* rand(2,1));
    parents(:,1,j) = top_performers(:,parent_indexes(1));
    parents(:,2,j) = top_performers(:,parent_indexes(2));
end

