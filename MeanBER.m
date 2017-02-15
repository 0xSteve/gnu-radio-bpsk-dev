
A=[];
SNR = (-20:5:30);
ebno = [2,7,12,17,22,27,32,37,42,47,52,57,62];
dist = (1:1:50);
for i=1:50
    
file=strcat('downward_refracting-',int2str(i),'/resultsfilters64POLY/BER');
s1=csvread(file);
%disp(s1(:,1));
A(:,i)=s1(:,1);
end
figure;
% berfit(ebno, A(:,:));
myMean = [];
for i = 1:50
    myMean = [myMean mean(A(:,i))];
end
hold on;
for i = 1:50
    i
    if ~(i == 1 ||  i == 19 || i == 23)
        berfit(1:13,A(:,i)');
    end
end
figure;
plot(dist, myMean)
