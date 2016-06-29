function [ out_signal ] = filt( input_signal, dt, f_cut )

    RC = 1 / (2*pi*f_cut);

    alpha = RC / (RC + dt);
    
    out_signal(1) = input_signal(1);
    
    input_signal = input_signal';
    
    for i=2:size(input_signal') 
        out_signal(i) = alpha * ( out_signal(i-1) + input_signal(i) - input_signal(i-1));
    end

        out_signal = out_signal';
end