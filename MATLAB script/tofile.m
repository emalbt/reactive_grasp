fileID = fopen('exp.txt','w');

for(i=1:length(vec))
    if(i<length(vec))
        fprintf(fileID,'%f,', vec(i));
    else
        fprintf(fileID,'%f', vec(i));
    end
end

fclose(fileID);