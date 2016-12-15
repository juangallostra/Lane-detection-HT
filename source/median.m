function z = median(C)
    C = sort(C);                % Sort pixel intensities
    n=length(C(:,1));           % Column length
    i=ceil(n/2);                % Compute median index       
    z=C(i,:);                   % return median              
end