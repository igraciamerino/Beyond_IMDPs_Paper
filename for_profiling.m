

% N = 1e7;
% 
% A = zeros(N,5,'uint8');
% b = zeros(N,1,'uint8');
% 
% for i = 1:N
%     A(i,:) = uint8([10 20 30 40 50]);
%     b(i) = uint8(ceil(5*rand(1)));
% end
tic
V = zeros(N,1,'uint8');

for i = 1:N
    V(i) = A(i,b(i));
end

% V = A(sub2ind(size(A),1:size(A,1),b'));

toc