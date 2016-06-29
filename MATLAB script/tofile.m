fileID = fopen('exp.txt','w');
for(i=1:length(vec))
    fprintf(fileID,'%f,', vec(i));
end
fclose(fileID);