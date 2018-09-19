function [nodes, springs, groups] = parse_mesh(filename)
% Read a mesh description from an xml file.
% Input: filename - file path (e.g. '/path/mesh.xml')
% Output: nodes - struct array of node info
%         springs - struct array of spring info
%        groups - struct array of control groups

try
   xRoot = xmlread(filename);
   xMesh = xRoot.getDocumentElement;
catch
   error('Failed to read XML file %s.', filename);
end

xNodes = xMesh.getElementsByTagName('node');
n = xNodes.getLength;
nodes = struct('static', false, 'x', cell(n,1), 'k', NaN, 'm', NaN);
for i = 1:n
    xNode = xNodes.item(i-1);
    
    % Node index
    idx = str2double(xNode.getAttribute('idx'));
    if isnan(idx) || (idx > n)
        error('Invalid node index %d.', idx)
    elseif ~isempty(nodes(idx).x)
        error('Repeat node index %d.', idx)
    end
    
    % Ignore if missing
    nodes(idx).k = str2double(xNode.getAttribute('k'));
    nodes(idx).m = str2double(xNode.getAttribute('m'));
    
    % Node position
    x = xNode.getElementsByTagName('x').item(0).getFirstChild.getData;
    nodes(idx).x = str2num(x)'; %#ok<ST2NM>
    if (numel(nodes(idx).x) ~= 3)
        error('Invalid node position %s.', x)
    end
    
    xStatic = xNode.getElementsByTagName('static');
    nodes(idx).static = (xStatic.getLength > 0);
end

xSprings = xMesh.getElementsByTagName('spring');
m = xSprings.getLength;
springs = struct('nodes', cell(m,1), 'rij0', [], 'rji0', [], 'l0', [], 'k', NaN, 'm', NaN);
for k = 1:m
    xSpring = xSprings.item(k-1);
    
    % Spring index
    idx = str2double(xSpring.getAttribute('idx'));
    if isnan(idx) || (idx > m)
        error('Invalid spring index %d.', idx)
    elseif ~isempty(springs(idx).nodes)
        error('Repeat spring index %d.', idx)
    end
    
    % Ignore if missing
    springs(idx).k = str2double(xSpring.getAttribute('k'));
    springs(idx).m = str2double(xSpring.getAttribute('m'));
    
    % Endpoint node indices
    node1 = str2double(xSpring.getElementsByTagName('node1').item(0).getFirstChild.getData);
    node2 = str2double(xSpring.getElementsByTagName('node2').item(0).getFirstChild.getData);
    if isnan(node1) || (node1 > n)
        error('Invalid node1 index %d.', node1)
    end
    if isnan(node2) || (node2 > n) || (node1 == node2)
        error('Invalid node2 index %d.', node2)
    end
    springs(idx).nodes = [node1 node2];
    
    % Default: springs are at rest in the mesh's initial configuration ...
    springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
    
    % ... unless otherwise specified
    xRij0 = xSpring.getElementsByTagName('rij0').item(0);
    if ~isempty(xRij0)
        springs(idx).rij0 = str2num(xRij0.getFirstChild.getData)'; %#ok<ST2NM>
    end
    xRji0 = xSpring.getElementsByTagName('rji0').item(0);
    if ~isempty(xRji0)
    	springs(idx).rji0 = str2num(xRji0.getFirstChild.getData)'; %#ok<ST2NM>
    end
    xL0 = xSpring.getElementsByTagName('l0').item(0);
    if ~isempty(xL0)
    	springs(idx).l0 = str2double(xL0.getFirstChild.getData);
    end
end

xGroups = xMesh.getElementsByTagName('group');
g = xGroups.getLength;
groups = struct('springs', cell(g,1));
for i = 1:g
    xGroup = xGroups.item(i-1);
    
    % Group index
    idx = str2double(xGroup.getAttribute('idx'));
    if isnan(idx) || (idx > g)
        error('Invalid group index %d.', idx)
    elseif ~isempty(groups(idx).springs)
        error('Repeat group index %d.', idx)
    end
    
    % Group actuators
    slist = xGroup.getElementsByTagName('springs').item(0).getFirstChild.getData;
    groups(idx).springs = str2num(slist)'; %#ok<ST2NM>
end

end