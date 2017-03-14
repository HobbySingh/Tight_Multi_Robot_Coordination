function successors=explore_successors2(x_cell,y_cell,h,x_target,y_target,in_valid,max_x,max_y,path)%,cell_x,cell_y)
    successors=[];
    dist_array = [];
    successor_count=1;
    c2=size(in_valid,1);
    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x = x_cell+k;
                s_y = y_cell+j;
                if( (s_x >0 && s_x <=max_x) && (s_y >0 && s_y <=max_y)) % successor within the matrix
                    flag=1;                    
                    for c1=1:c2
                        if(s_x == in_valid(c1,1) && s_y == in_valid(c1,2)) % successor not an obstacle or already visited
                            flag=0;
                        end;
                    end;
                    if (flag == 1)
                        [path_size,~] = size(path); 
                        successors(successor_count,1) = s_x;
                        successors(successor_count,2) = s_y;
                        successors(successor_count,3) = h+pdist([x_cell,y_cell;s_x,s_y],'euclidean');%h
                        successors(successor_count,4) = pdist([x_target,y_target;s_x,s_y],'euclidean');%g
                        for count = 1:path_size
                            cell_x = path(count,1);
                            cell_y = path(count,2);
                            dist_array(count) = pdist([s_x,s_y;cell_x,cell_y],'euclidean');
                        end
                        min(dist_array);
                        if(min(dist_array) < 4.1)
                            successors(successor_count,5) = successors(successor_count,3)+successors(successor_count,4);%f
                        else
                            successors(successor_count,5) = successors(successor_count,3)+successors(successor_count,4) + 1000;%f
                            disp('coordination broken');
                        end
                        successor_count=successor_count+1;
                    end
                end
            end
        end
    end