clear all
close all
clc


file_name = 'trace_shm1.txt';




T = table;
count = 1;
fid = fopen(file_name);
tline = fgetl(fid);
while ischar(tline)
 
    if tline(1) ~= '#'
        %disp(tline);
        tline = regexprep(tline,'\s+',' ');
        tline = regexprep(tline,'capture thread','capture_thread');
        tline = regexprep(tline,'JS Watchdog','JS_Watchdog');
        tline = regexprep(tline,'JS Helper','JS_Helper');
        
        
        newStr = split(tline,' ');
        
        task_pid  = newStr{2};
        cpu_val   = newStr{3};
        bits_val  = newStr{4};
        timestamp_val = newStr{5};
        function_name = newStr{6};
        
        prev_comm  = split(newStr{7},'=');
        prev_pid   = split(newStr{8},'=');
        prev_prio  = split(newStr{9},'=');
        prev_state = split(newStr{10},'=');
        
        next_comm = split(newStr{12},'=');
        next_pid  = split(newStr{13},'=');
        next_prio = split(newStr{14},'=');
        
        timestamp_val = str2double(timestamp_val(1:end-1));
        cpu_val = str2double(cpu_val(2:end-1)); 
        function_name = function_name(1:end-1);
        prev_comm = prev_comm{2};
        prev_pid = str2double(prev_pid{2});
        prev_prio = str2double(prev_prio{2});
        prev_state = prev_state{2};
        next_comm = next_comm{2};
        next_pid = str2double(next_pid{2});
        next_prio = str2double(next_prio{2});
       
        T{count,:} = {task_pid, cpu_val, bits_val, timestamp_val,function_name,prev_comm,prev_pid,prev_prio,prev_state,next_comm,next_pid,next_prio};
        count= count+1;
        
         
    end
    tline = fgetl(fid);
end
fclose(fid);

save('trace.mat','T')


