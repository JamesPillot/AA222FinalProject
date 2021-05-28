function [new_population] = gaussianMutation(children, sigma, population_size, lowerbound)
% Takes in children and applies Gaussian Mutation on genes, returns
% new population with mutated genes
new_population = children;
for i = 1:population_size
    new_population(:,i) = new_population(:,i) + randn(5,1)*sigma;
    new_population(:,i) = gene_check(new_population(:,i), lowerbound);
end

