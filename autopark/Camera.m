classdef Camera < matlab.System
% 按深度和视野参数化的相机表示

    % 可调整
    properties
        MaxDepth = 10
        MaxViewAngle = pi/2
    end
    
    properties (Nontunable)
        % 地图对象名称
        MapObject char
    end

    properties(DiscreteState)
        Status
    end

    properties(Access = private)
        %Map
    end

    methods(Access = protected)
        function setupImpl(obj)
            % 执行一次性计算，例如计算常量
            %obj.Map = evalin('base',obj.MapObject);
        end

        function [isfree,targetpose] = stepImpl(obj,pose)
            % 实现算法，用输入 U 和离散函数计算y。
            
            map = evalin('base',obj.MapObject);
            
            % 获取点位
            loc      = map.SensorLocations;
            occupied = map.OccupiedSpots;
            
            % 额外数据.
            locx        = loc(:,1);
            locy        = loc(:,2);
            carx        = pose(1);
            cary        = pose(2);
            cartheta    = mod(pose(3), 2*pi);
            phi         = atan2( (locy-cary) , (locx-carx) );  
            phi         = mod(phi, 2*pi); % phi 是传感器灯在 [0，2*pi]内的角度
            phimin      = mod(cartheta - 0.5*obj.MaxViewAngle, 2*pi);
            phimax      = phimin + obj.MaxViewAngle;
            
            % 计算所有点的有效角度
            for i = 1:numel(phi)
                if phimax >= 2*pi && phi(i) <= pi/2
                    phi(i) = 2*pi + phi(i);
                end
            end
            validPhiIdx = (phi >= phimin) & (phi <= phimax);
            
            % 计算与每个点的有效距离
            dist = sqrt( (locx-carx).^2 + (locy-cary).^2 );
            validDistIdx = dist <= obj.MaxDepth;
            
            spotsToCheck = validPhiIdx & validDistIdx;
            freeSpots = find(spotsToCheck' & ~occupied, map.MaxSpots);
            targetSpot = min(freeSpots);
            
            isfree = double(~isempty(targetSpot));
            spotfound = obj.Status(1);
            if isfree && spotfound==0
                targetpose  = createTargetPose(map,targetSpot);
                obj.Status = [1 reshape(targetpose, 1, 3)];
            end
            
            targetpose = obj.Status(2:end);
            
        end

        function resetImpl(obj)
            % 初始化/重置离散状态属性
            obj.Status = [0 0 0 0];
        end

        function ds = getDiscreteStateImpl(obj)
            % 返回具有离散属性的性能结构
            ds.Status  = obj.Status;
        end

        function validateInputsImpl(obj,pose)
            % 在初始化时验证方法的输入
            validateattributes(pose,{'double'},{'numel',3});
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % 如果输入大小不能在对系统对象的调用之间更改，则返回false
            flag = false;
        end

        function flag = isInputComplexityMutableImpl(obj,index)
            % 如果输入复杂度在对 System 对象的调用之间无法更改，则返回 false
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % 使用可选输入定义系统的输入总数
            num = 1;
            % if obj.UseOptionalInput
            %     num = 2;
            % end
        end

        function num = getNumOutputsImpl(obj)
            % 定义具有可选输出的系统的输出总数
            num = 2;
            % if obj.UseOptionalOutput
            %     num = 2;
            % end
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = "Camera";
            % icon = ["My","System"]; % Example: multi-line text icon
        end

        function name = getInputNamesImpl(obj)
            % Return input port names for System block
            name = 'pose';
        end

        function [name,name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = 'found';
            name2 = 'goal';
        end

        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
            out2 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 4];
            dt = "double";
            cp = false;
        end

        
        
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"),...
           'Title','Camera',...
           'Text', 'This system object models a camera for identifying free parking spots.');
        end
        
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
