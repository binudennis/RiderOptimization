function [fit]=fitness_function(pop, dim, indiv)

for i=1:indiv
fit(i)=sum(pop(:,i).^2);

end


end