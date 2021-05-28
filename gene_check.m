function [chromosome] = gene_check(chromosome,lowerbound)
% Takes in a chromosome and makes sure all the genes have legal values
for i = 1:5 % number of genes in chromosome
    if(chromosome(i) <= 0)
        chromosome(i) = lowerbound;
    end
end
end

