function map(coordinates)
    i = 10;
    coordinates = (coordinates+1)*i;
    grid_world = ones(140,12000);
    maps = ones(1,3);
    grid_world = ind2rgb(grid_world,maps);
    for i = 1:size(coordinates,1)
        grid_world(coordinates(i,2)+1:coordinates(i,2)+9,coordinates(i,1)+1:coordinates(i,1)+9,1) = 0 + 0.1 * i;
        grid_world(coordinates(i,2)+1:coordinates(i,2)+9,coordinates(i,1)+1:coordinates(i,1)+9,2) = 0 + 0.12 * i;
        grid_world(coordinates(i,2)+1:coordinates(i,2)+9,coordinates(i,1)+1:coordinates(i,1)+9,3) = 1 - 0.1 * i;
    end
    imagesc(grid_world)

%     hold on
%     % M = size(grid_world,1);
%     % N = size(grid_world,2);
%     for k = 10:10:130
%         x = [10 11990];
%         y = [k k];
%         plot(x,y,'Color','black','LineStyle','-','LineWidth',4);
%         %     set(findobj('Tag','MyGrid'),'Visible','on')
%     end
%     for k = 10:10:11990
%         x = [k k];
%         y = [10 130];
%         plot(x,y,'Color','black','LineStyle','-','LineWidth',4);
%         %     set(findobj('Tag','MyGrid'),'Visible','on')
%     end
    