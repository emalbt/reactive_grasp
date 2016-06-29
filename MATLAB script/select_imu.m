function [ out ] = select_imu( M, id )
    
    n_imu = length(M)/3;
    
    if(id>n_imu)
        display('Error: id>n_imu');
        out = 0;
    else
        out = [M(:,id*3+1), M(:,id*3+2), M(:,id*3+3)];
    end
    
end

