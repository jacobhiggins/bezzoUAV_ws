close all
clear all
clc
load('trace.mat')

cpu_key = 2;
timestamp_key = 4;
next_task_key = 10;
next_pid_key = 11;
prev_task_key = 6;
prev_pid_key = 7;

START = 1;
STOP = 2;

num_cpu = length(unique(cell2mat(T{:,cpu_key})));
cpu_ids = unique(cell2mat(T{:,cpu_key}));


num_lines               = size(T,1);
[all_tasks, all_ids]    = unique(T{:,prev_task_key});
all_pids                = cell2mat(T{all_ids,prev_pid_key});

matlab_tasks_i          = startsWith(all_tasks,"MATLAB");
mpc_tasks_i             = startsWith(all_tasks,"mpc_");

all_tasks_indexes       = matlab_tasks_i | mpc_tasks_i;
tasks_names             = all_tasks(all_tasks_indexes);
task_pids               = all_pids(all_tasks_indexes);

num_tasks = sum(all_tasks_indexes);
tasks_info = zeros(num_tasks,2);

y_bottom = (1:1:num_tasks)./1000;
y_top = ((1:1:num_tasks)+0.8)./1000


figure(1)
hold on
for i = 1 : num_lines
    % look in the next_pid row
    [isInTheTable,atIndex] = ismember(cell2mat(T{i,next_pid_key}),task_pids);
    % if there is a task of interest
    if isInTheTable
      
            %get cpu, get time
            cpu = cell2mat(T{i,cpu_key});
            t = cell2mat(T{i,timestamp_key});
            % update task_ifo with start and save time
            tasks_info(atIndex,1) = START;
            tasks_info(atIndex,2) = t;
            % draw according to cpu
            if cpu == 0
                marker = 'k-';
            else
                if cpu == 1
                    marker = 'r-';
                else
                    marker = 'm-';
                end
            end      
            plot([t,t],[ (y_bottom(atIndex)) y_top(atIndex)],marker) 
        
    end
    % look in the prev_pid row
    [isInTheTable,atIndex] = ismember(cell2mat(T{i,prev_pid_key}),task_pids);
    % if there is a task of interest
    if isInTheTable
        % if this task was running
        if  tasks_info(atIndex,1) ~= 0
            if tasks_info(atIndex,1) == START
            %get cpu, get time
            cpu = cell2mat(T{i,cpu_key});
            t = cell2mat(T{i,timestamp_key});
     
            % then stop. and set t
            tasks_info(atIndex,1) = STOP;
            t_old = tasks_info(atIndex,2);
            tasks_info(atIndex,2) = t;
            %draw
            
            % draw according to cpu
            if cpu == 0
                marker = 'k-';
            else
                if cpu == 1
                    marker = 'r-';
                else
                    marker = 'm-';
                end
            end
            plot([t,t],    [ y_bottom(atIndex) y_top(atIndex) ],'b-')
            plot([t_old,t],[ y_top(atIndex)    y_top(atIndex) ],marker)
            else
                i
                keyboard
            end
        end
    end
end

t_min = min(cell2mat(T{:,timestamp_key}));
t_max = max(cell2mat(T{:,timestamp_key}));

for i = 1 :  num_tasks
 %  text(t_min,y_bottom(i), tasks_names{i});
    plot([t_min,t_max],[y_bottom(i) y_bottom(i)],'k-')
end

yticks(y_top)
yticklabels(tasks_names)


return;
 




