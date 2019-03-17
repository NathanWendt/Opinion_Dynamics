function [count] = RandomWalks(A, start, steps)
    n = size(A,1);
    degO=zeros(1,n);
    E{1,n} = [];  
    for k=1:n
       outs = find(A(k,:));
       degO(k) = size(outs,2);
       E{k} = outs;
    end

    current = start;
    count = zeros(1,n);
    for i = 1:steps
        deg = degO(current);
        next = randi(deg);
        current = E{current}(next);
        count(current) = count(current)+1;
    end
end
