load('A_dolph');
load('G_dolph');
average_time = zeros(1,length(A));
for i = 1:length(A)
    temp_avg = 0;
    for j = 1:100
        itters = itters2conv(i,0,A,G_dolph);
        temp_avg = temp_avg + itters;
    end
    average_time(i) = temp_avg/100;
end



diG = digraph(A);

average_time = average_time'.^-1;
average_time = average_time/max(average_time);

pg_ranks = centrality(diG,'pagerank');
pg_ranks = pg_ranks/max(pg_ranks);
dis = abs(average_time - pg_ranks);                            
mean_sqr_PR = mean(dis.^2);                          


indeg = centrality(diG,'indegree');
indeg = indeg/max(indeg);
dis = abs(average_time - indeg);                            
mean_sqr_indeg = mean(dis.^2);  

outdeg = centrality(diG,'outdegree');
outdeg = outdeg/max(outdeg);
dis = abs(average_time - outdeg);                            
mean_sqr_outdeg = mean(dis.^2);  

inclose = centrality(diG,'incloseness');
inclose = inclose/max(inclose);
dis = abs(average_time - inclose);                            
mean_sqr_inclose = mean(dis.^2);  

outclose = centrality(diG,'outcloseness');
outclose = outclose/max(outclose);
dis = abs(average_time - outclose);                            
mean_sqr_outclose = mean(dis.^2);  

between = centrality(diG,'betweenness');
between = between/max(between);
dis = abs(average_time - between);                           
mean_sqr_between = mean(dis.^2);  



%%

c = categorical({'Pagerank','RWS','In Degree','Out Degree','In Closeness','Out Closeness','Betweenness'});
y = [mean_sqr_PR; mean_sqr_RWS; mean_sqr_indeg; mean_sqr_outdeg; mean_sqr_inclose; mean_sqr_outclose; mean_sqr_between];
x_ax = 1:1:length(pg_ranks);
figure(8)
bar(c,y)
title('Mean Square Distance Between Normalized Convergence Time vs Centrality Metrics')

figure(1)
plot(x_ax,pg_ranks,x_ax,average_time,x_ax,indeg,x_ax,outdeg,x_ax,inclose,x_ax,outclose,x_ax,between)
legend('PR','Average Time','indegree','outdegree','incloseness','outcloseness','betweenness')

figure(2)
p = plot(x_ax,average_time,x_ax,pg_ranks)
legend('Average Time','PR')
set(p,'LineWidth',1.5)
set(gca,'FontSize',15,'LineWidth',1.2)
xlabel('Node')
ylabel('Normalized Value')

scatter(x_ax,average_time)
hold on
scatter(x_ax,pg_ranks)
figure(3)
plot(x_ax,average_time,x_ax,indeg)
legend('Average Time','indegree')

figure(4)
plot(x_ax,average_time,x_ax,outdeg)
legend('Average Time','outdegree')

figure(5)
plot(x_ax,average_time,x_ax,inclose)
legend('Average Time','incloseness')

figure(6)
plot(x_ax,average_time,x_ax,outclose)
legend('Average Time','outcloseness')

figure(7)
plot(x_ax,average_time,x_ax,between)
legend('Average Time','betweenness')