close all
clear all
clc
load('trace.mat')

% the values stored in the table.
cpu_key         = 2;
timestamp_key   = 4;
next_task_key   = 10;
next_pid_key    = 11;
prev_task_key   = 6;
prev_pid_key    = 7;

% value assigned to know if the task is started or stopped
NEVERSTARTED    = 0;
STARTED         = 1;
STOPPED         = 2;
 
% get number of lines of the table.
num_lines               = size(T,1);

% get all the cps used and their ids
num_cpu = length(unique(cell2mat(T{:,cpu_key})));
cpu_ids = unique(cell2mat(T{:,cpu_key}));

% get all the pid and all tasks names by looking at prev_task_key
[all_tasks, all_ids]    = unique(T{:,prev_task_key});
all_pids                = cell2mat(T{all_ids,prev_pid_key});

% look for the tasks that started with the name MATLAB and MPC
matlab_tasks_i          = startsWith(all_tasks,"MATLAB");
mpc_tasks_i             = startsWith(all_tasks,"mpc_");

% get all tasks indexes that we are interested in and get their names and
% pids
all_tasks_indexes       = matlab_tasks_i | mpc_tasks_i;
tasks_names             = all_tasks(all_tasks_indexes);
task_pids               = all_pids(all_tasks_indexes);

% get number of tasks we are gonna trace
num_tasks = sum(all_tasks_indexes); 

% variables used to store info
tasks_info = zeros(num_tasks,2);
tasks_log = {};

%init the struct
for i = 1 : num_tasks
     tasks_log(i).num_executions = 0;
end
     
%for each line
for i = 1 : num_lines
    % look in the next_pid row
    [isInTheTable,atIndex] = ismember(cell2mat(T{i,next_pid_key}),task_pids);
    % if there is a task of interest
    if isInTheTable
            %get cpu, get time
            cpu = cell2mat(T{i,cpu_key});
            t = cell2mat(T{i,timestamp_key});
            tasks_info(atIndex,1) = STARTED;
            % store the execution
            num_executions =  tasks_log(atIndex).num_executions + 1;
            tasks_log(atIndex).num_executions = num_executions;
            tasks_log(atIndex).time_init(num_executions) = t;
            tasks_log(atIndex).cpu_at_execution(num_executions) = cpu;
          
    end
    % look in the prev_pid row
    [isInTheTable,atIndex] = ismember(cell2mat(T{i,prev_pid_key}),task_pids);
    % if there is a task of interest
    if isInTheTable
        % if this task was running
        if  tasks_info(atIndex,1) ~= NEVERSTARTED
            if tasks_info(atIndex,1) == STARTED
                tasks_info(atIndex,1) = STOPPED;
                %get cpu, get time
                t = cell2mat(T{i,timestamp_key});
                num_executions =  tasks_log(atIndex).num_executions;
                tasks_log(atIndex).time_end(num_executions) = t;
            end
        end
    end
end

t_min = min(cell2mat(T{:,timestamp_key}));
t_max = max(cell2mat(T{:,timestamp_key}));
 
task_id = 2;

figure(1)
title('execution time')
for i = 1 : num_tasks
    subplot(num_tasks,1,i)
    num_executions =  min(  tasks_log(i).num_executions, length(tasks_log(i).time_end) );
    values = tasks_log(i).time_end(1:num_executions) - tasks_log(i).time_init(1:num_executions);
    plot(1:1:num_executions,values,'-o')
    xlabel('executions')
    ylabel('execution time')
   
end

figure(2)
title('schedule')
hold on
for i = 1 : num_tasks
    num_executions =  min(  tasks_log(i).num_executions, length(tasks_log(i).time_end) );
    for j = 1 : num_executions
        t_init = tasks_log(i).time_init(j);
        t_end = tasks_log(i).time_end(j);
        cpuval = tasks_log(i).cpu_at_execution(j);
       
        rectangle('Position',[t_init i (t_end-t_init) 0.8])
    end
    plot([t_min,t_max],[i i],'k-')
    xlabel('time')
    ylabel(tasks_names(i))
    %axis([t_min t_max 0 2])
end


% 
% 
% variables used for plotting
% y_bottom = (1:1:num_tasks)./1000;
% y_top = ((1:1:num_tasks)+0.8)./1000;
% figure(1)
% hold on
% 
%     the computation time of each execution.
%     start and stop for each execution.
%     the mean rate?
%     
% for i = 1 : num_lines
%     look in the next_pid row
%     [isInTheTable,atIndex] = ismember(cell2mat(T{i,next_pid_key}),task_pids);
%     if there is a task of interest
%     if isInTheTable
%             get cpu, get time
%             cpu = cell2mat(T{i,cpu_key});
%             t = cell2mat(T{i,timestamp_key});
%             update task_ifo with start and save time
%             tasks_info(atIndex,1) = STARTED;
%             tasks_info(atIndex,2) = t;
%             
%             tasks_log{atIndex}.time_init(end+1) = t;
%             tasks_log{atIndex}.num_executions = tasks_log{atIndex}.num_executions + 1;
%             
%             
%             draw according to cpu
%             if cpu == 0
%                 marker = 'k-';
%             else
%                 if cpu == 1
%                     marker = 'r-';
%                 else
%                     marker = 'm-';
%                 end
%             end      
%             plot([t,t],[ (y_bottom(atIndex)) y_top(atIndex)],marker) 
%         
%     end
%     look in the prev_pid row
%     [isInTheTable,atIndex] = ismember(cell2mat(T{i,prev_pid_key}),task_pids);
%     if there is a task of interest
%     if isInTheTable
%         if this task was running
%         if  tasks_info(atIndex,1) ~= NEVERSTARTED
%             if tasks_info(atIndex,1) == STARTED
%             get cpu, get time
%             cpu = cell2mat(T{i,cpu_key});
%             t = cell2mat(T{i,timestamp_key});
%      
%             then stop. and set t
%             tasks_info(atIndex,1) = STOPPED;
%             t_old = tasks_info(atIndex,2);
%             tasks_info(atIndex,2) = t;
%             
%             count = counters(atIndex);
%             tasks_log(atIndex,count) = t;
%             counters(atIndex) = count +1 ;
% 
%             draw
%             
%             draw according to cpu
%             if cpu == 0
%                 marker = 'k-';
%             else
%                 if cpu == 1
%                     marker = 'r-';
%                 else
%                     marker = 'm-';
%                 end
%             end
%             plot([t,t],    [ y_bottom(atIndex) y_top(atIndex) ],marker)
%             plot([t_old,t],[ y_top(atIndex)    y_top(atIndex) ],marker)
%             else
%                 i
%                 keyboard
%             end
%         end
%     end
% end

% t_min = min(cell2mat(T{:,timestamp_key}));
% t_max = max(cell2mat(T{:,timestamp_key}));
% 
% for i = 1 :  num_tasks
%   text(t_min,y_bottom(i), tasks_names{i});
%     plot([t_min,t_max],[y_bottom(i) y_bottom(i)],'k-')
% end



% yticks(y_top)
% yticklabels(tasks_names)

 

% cpu_executions = tasks_log(task_id ,2:2:counters(task_id)) - tasks_log(2,1:2:counters(task_id)-1)


return;
 




