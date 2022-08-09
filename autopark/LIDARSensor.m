classdef LIDARSensor < matlab.System
% 雷达传感器模型

    properties
        % 矩形的全局位置和方向，指定为N x 5
        % vector [x,y,L,W,theta]
        %Rectangles = [5,5,1,3,0;5,-5,1,3,0]
    end
    properties (Nontunable)
        % MapObject Name of ParkingLot object
        MapObject = ''
        
        % 汽车的几何结构[L,W]
        CarGeometry = [3,1]
        
        % 传感器的最大距离读数
        MaxDistance = 12
        % 可用的传感器读数数量
        SensorResolution (1, 1) {mustBePositive, mustBeInteger} = 60; %numObsLidar
    end
    properties
        % 显示传感器图
        ShowLIDARPlot = 0
    end
    properties
        % Limits of the plot [xLower,xUpper,yLower,yUpper]
        PlotLimits = [-20,20,-20,20]
    end
    properties (Access = private)
        Rectangles
    end
    methods 
        function this = LIDARSensor(varargin)
            setProperties(this,nargin,varargin{:});
        end
    end
    methods (Access = protected)
        function d = outputImpl(this,botx,boty,bottheta)
            
            % 获取数据
            sensorRes = this.SensorResolution;
            maxDistance = this.MaxDistance; % meters
            
            % 矩形中创建分段
            %rect = this.Rectangles(~all(this.Rectangles==zeros(1,5),2),:);
            map = evalin('base', this.MapObject);
            rect = map.ObstacleMatrix;
            [segsx,segsy] = rect2segs(rect);
            
            % 变换给定机器人位置的线段
            s = sin(bottheta); c = cos(bottheta);
            R = [c,-s;s,c];
            
            x_ = segsx(:)';
            y_ = segsy(:)';
            
            z = (R')*([x_;y_] - [botx;boty]);
            
            segsx_xform = reshape(z(1,:)',size(segsx));
            segsy_xform = reshape(z(2,:)',size(segsy));
            
            % 生成雷达数据
            [d,intersections,~,lidarx,lidary] = lidarSegmentIntersections(...
                sensorRes,maxDistance,segsx_xform,segsy_xform);
            
            if this.ShowLIDARPlot
                plotLIDAR(this,botx,boty,bottheta,R,segsx,segsy,lidarx,lidary,intersections);
            end
        end
        function updateImpl(this,x,y,theta) %#ok<INUSD>
        end
        function setupImpl(this)
            map = evalin('base', this.MapObject);
            this.Rectangles = map.ObstacleMatrix;
        end
        function resetImpl(this) %#ok<MANU>
        end
        function plotLIDAR(this,botx,boty,bottheta,R,segsx,segsy,lidarx,lidary,intersections)
            persistent f
            if isempty(f) || ~isvalid(f)
%                 f = figure(...
%                     'Toolbar','none',...
%                     'NumberTitle','off',...
%                     'Name','LIDAR Sensor',...
%                     'MenuBar','none');
                f = figure('NumberTitle','off',...
                    'Name','LIDAR Sensor');
                ha = gca(f);
                grid(ha,'on');
            end
            
            ha = gca(f);
            
            ha.XLim = this.PlotLimits(1:2);
            ha.YLim = this.PlotLimits(3:4);
            
            % 获取用于绘制的小车距离
            res = size(lidarx,1);
            angles = ((0:(res-1))*2*pi/res)';
            cd = getCarSegmentLengths(this.CarGeometry(1),this.CarGeometry(2),angles);
            
            cdx = cd.*cos(angles);
            cdy = cd.*sin(angles);
            
            lidarx(:,1) = cdx(:);
            lidary(:,1) = cdy(:);
            
            % xform the lidar data for plotting
            x_ = lidarx(:)';
            y_ = lidary(:)';
            
            z = R*[x_;y_] + [botx;boty];
            
            lidarx_xform = reshape(z(1,:)',size(lidarx));
            lidary_xform = reshape(z(2,:)',size(lidary));
            
            ix = [];
            for i = 1:numel(intersections)
                intersection = intersections{i};
                if ~isempty(intersection)
                    intersection = (R*intersection' + [botx;boty])';
                    ix(end+1,:) = intersection; %#ok<AGROW>
                end
            end
            
            % plot
            lh = findobj(ha,'Tag','seg_lh');
            if isempty(lh) || numel(lh) ~= size(segsx,1)
                delete(lh);
                line(ha,segsx',segsy','Color','k','Tag','seg_lh');
            else
                for i = 1:numel(lh)
                    set(lh(i),'XData',segsx(i,:)','YData',segsy(i,:)');
                end
            end

            lh = findobj(ha,'Tag','lidar_lh');
            if isempty(lh) || numel(lh) ~= this.SensorResolution
                delete(lh);
                % lidar objs
                lh = line(ha,lidarx_xform',lidary_xform','Color','g','Tag','lidar_lh');
                % 第一行表示方向
                lh(1).Color = 'k';
            else
                % 翻转以保持图形顺序
                lh = flip(lh);
                for i = 1:this.SensorResolution
                    set(lh(i),'XData',lidarx_xform(i,:)','YData',lidary_xform(i,:)');
                end
            end
            
            intersection_lh = findobj(ha,'Tag','intersect_lh');
            delete(intersection_lh);
            if ~isempty(ix)
                line(ha,ix(:,1),ix(:,2),'Color','r','Marker','x','LineStyle','none','Tag','intersect_lh'); 
            end
            
            % 绘制小车
            [carsegsx,carsegsy] = rect2segs([botx,boty,this.CarGeometry,bottheta]);
            lh = findobj(ha,'Tag','car_lh');
            if isempty(lh) || numel(lh) ~= size(carsegsx,1)
                delete(lh);
                line(ha,carsegsx',carsegsy','Color','k','LineStyle','-','Tag','car_lh');
            else
                for i = 1:numel(lh)
                    set(lh(i),'XData',carsegsx(i,:)','YData',carsegsy(i,:)');
                end
            end
            
        end
        function releaseImpl(~)
        end

        function num = getNumInputsImpl(~)
            % 定义具有可选输入的系统的输入总数
            num = 3;
        end
        function num = getNumOutputsImpl(~)
            % 定义带有可选参数的系统的输出总数
            % 输出
            num = 1;
        end
        function loadObjectImpl(this,s,wasLocked)
            % 需要快速重启
            
            % 设置公共属性和状态
            loadObjectImpl@matlab.System(this,s,wasLocked);
        end

        function s = saveObjectImpl(this)
            
            s = saveObjectImpl@matlab.System(this);
        end

        function varargout = getInputNamesImpl(~)
            
            varargout{1} = 'botx';
            varargout{2} = 'boty';
            varargout{3} = 'theta';
        end

        function varargout = getOutputNamesImpl(~)
            
            varargout{1} = 'd';
        end
        function varargout = isInputDirectFeedthroughImpl(this,varargin)
            % 当TreatAsDirectFeedthrough为false时，隐式单元延迟被添加到输入信号。在大多数情况下TreatAsDirectFeedthrough应为true
            
            nu = nargin(this);
            varargout = cell(1,nu);
            for i = 1:nu
                varargout{i} = true;
            end
        end
        function varargout = getOutputSizeImpl(this)
            % 从agent处获取尺寸和采样时间
            varargout{1} = [this.SensorResolution,1];
        end
        function varargout = isOutputComplexImpl(this)
            ny = nargout(this);
            varargout = cell(1,ny);
            for i = 1:ny
                varargout{i} = false;
            end
        end
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'double';
        end
        function varargout = isOutputFixedSizeImpl(this)
            ny = nargout(this);
            varargout = cell(1,ny);
            for i = 1:ny
                varargout{i} = true;
            end
        end
    end
    methods(Access = protected,Static)
        function simMode = getSimulateUsingImpl
            
            simMode = "Interpreted execution";
        end
    end
end
