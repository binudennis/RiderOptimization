clc;
clear all;
close all;

%%%%%%Initialization%%%%%%%%%%%%%%%%%
iter_max = 1000;%set how many iterations
indiv=10;
dim=2;
num_gear=5;
obj=0; % 0 for minimization and 1 for maximization

%%Steering position
for i=1:indiv
   pos_dir(i)=i*360/indiv;
 end
cor_dir=360/(dim);

for i=1:indiv
for j=1:dim
       if(j==1)
           x_steer_new(i, j)=pos_dir(i);
       else
           x_steer_new(i, j)=x_steer_new(i, j-1)+cor_dir;
       if(x_steer_new(i, j)>=360)
           x_steer_new(i, j)=x_steer_new(i, j)-360;
       end
       end
end
end
%%%%%%%%%%%%%%%
pop=rand(dim,indiv); 
x_gear=zeros(1, indiv);
per_aux=zeros(1, indiv);
per_brake=zeros(1, indiv);
improv_counter=zeros(1, indiv);
Max_speed=(max(pop)-min(pop))/iter_max;
speed_thre_gear=Max_speed/num_gear;

%**************************************************************************
%Loop over generations
%********************   ******************************************************

algo=1;
for(algo=1:iter_max)
         time_norm=1/iter_max;
         fitness=fitness_function(pop, dim, indiv);
         if (obj==1)
             [va, leader_inx]=max(fitness);
         else
             [va, leader_inx]=min(fitness);    
         end
         %%%%%%%%%%% Bypass rider
         for k=1:indiv  
            if(rem(k,4)==1)  
               for j=1:size(pop,1) 
                   deta=rand;
                   eta=randi(indiv);
                   ceta=randi(indiv);
                   beta=rand(1, dim); 
                   pop_new(j,k)=deta*((pop(j, eta)*beta(j))+(pop(j,ceta)*(1-beta(j))));
              end
            %%%%%%%%%%%%Follower%%%%%%%%%%%%%%
            elseif(rem(k,4)==2)  
            fact_rate=(algo/iter_max)*dim;
            pop_new(:,k)=pop(:,k);
            inx_change=ceil(1:fact_rate:dim);
            inx_change=unique(inx_change);
            for jj=1:size(inx_change, 2)
                speed=((x_gear(1,k)*speed_thre_gear(1,k))+(Max_speed(1,k)*per_aux(1,k))+(Max_speed(1,k)*(1-per_brake(1,k))))/3;
                distance=speed*time_norm;
                pop_new(inx_change(jj),k)=pop(inx_change(jj), leader_inx)+(distance.*cos(x_steer_new(k, j))*pop(inx_change(jj),leader_inx));
            end
            %%%%%%%%%%%%Overtaker
            elseif(rem(k,4)==3)  
                fit_rate=fitness(k)/max(fitness);
                fit_chnage=(2/(1-log(fit_rate)))-1;
                pop_new(:,k)=pop(:,k);
                inx_c=abs(pop_new(:,k)-pop(:,leader_inx));
                ix_d=inx_c<(mean(inx_c));
                inx_change_2=find(ix_d==1);
                %%%%empty handling
                TF = isempty(inx_change_2);
                if(TF==1)
                    fit_rate_1=dim-fit_rate;
                    inx_change_2=ceil(1:fit_rate_1:dim);
                end
            for jj=1:size(inx_change_2, 2)
                pop_new(inx_change_2(jj),k)=pop(inx_change_2(jj),k)+(fit_chnage.*pop(inx_change_2(jj), leader_inx));
            end
            else
            for j=1:size(pop,1) 
         %%%%%%%%%%%%Attacker
              speed=((x_gear(1,k)*speed_thre_gear(1,k))+(Max_speed(1,k)*per_aux(1,k))+(Max_speed(1,k)*(1-per_brake(1,k))))/3;
              distance=speed*time_norm;
              pop_new(j,k)=pop(j, leader_inx)+(cos(x_steer_new(k, j))*pop(j,leader_inx))+distance;
            end
            end
         end
        %Improvement vector finding
         fitness_new=fitness_function(pop_new, dim, indiv);
         if (obj==1)
             improv_counter=(fitness_new>fitness);
         else
             improv_counter=(fitness_new<fitness);     
         end
         %Update the parameters
         x_st=x_steer_new;
         %Steering vector update
         for i=1:indiv
                if(improv_counter(1, i)==1)
                    if(i~=indiv)
                        x_st(i, :)=x_steer_new(i+1,:);
                    else
                        x_st(i, :)=x_steer_new(1,:);
                    end
          
                else
                   if(i~=1)
                      x_st(i, :)=x_steer_new(i-1,:);
                   else
                      x_st(i, :)=x_steer_new(end,:);
                   end
          
                end
         end
         x_steer_new=x_st;
         %Gear change
         for i=1:indiv
                gear_counter=0;
                if(improv_counter(1, i)==0)
                    if(x_gear(1,i)==0)
                       x_gear(1,i)=x_gear(1,i);
                    else
                       x_gear(1,i)=x_gear(1,i)-1;
                    end
                else
                    x_gear(1,i)=x_gear(1,i)+1;
                end
                %Normaizing the gear
                if(x_gear(1,i)>num_gear)
                   x_gear(1,i)=num_gear;
                end
                %Aux update
                per_aux(1, i)=(x_gear(1,i)/num_gear);
                %Brake update
                per_brake(1, i)=1-(x_gear(1,i)/num_gear);  
         end
         
         pop=pop_new;
         if (obj==1)
             best(algo)=max(fitness);
         else
             best(algo)=min(fitness);   
         end
 end
          
 
