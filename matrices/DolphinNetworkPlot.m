load('A_dolph');
load('G_dolph');
diG = digraph(A);

outdeg = centrality(diG,'outdegree');
pagerank = centrality(diG,'pagerank');
outdeg = 7*outdeg/max(outdeg)+1;
pagerank = 8*pagerank/max(pagerank);
figure(1)
h1 = plot(G_dolph,'Layout','Force','NodeLabel',{});
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
box off
figure(2)
h2 = plot(G_dolph,'Layout','Force','NodeLabel',{});
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
box off
for i = 1:length(outdeg)
    highlight(h1,[i],'MarkerSize',outdeg(i))
    highlight(h2,[i],'MarkerSize',pagerank(i))
end