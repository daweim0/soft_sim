function save_mesh(filename, nodes, springs, groups)
% Write a mesh description to an xml file.
% Input: filename - file path (e.g. '/path/mesh.xml')
%        nodes - struct array of node info
%        springs - struct array of spring info
%        groups - struct array of control groups

xRoot = com.mathworks.xml.XMLUtils.createDocument('mesh');

xMesh = xRoot.getDocumentElement;
xMesh.setAttribute('name', 'my_mesh');

for i = 1:numel(nodes)
    xNode = xRoot.createElement('node');
    
    % Node attributes
    xNode.setAttribute('idx', num2str(i));
    xNode.setAttribute('k', num2str(nodes(i).k));
    xNode.setAttribute('m', num2str(nodes(i).m));
    
    % Static indicator element
    if nodes(i).static
        xNode.appendChild(xRoot.createElement('static'));
    end
    
    % Node position
    xX = xRoot.createElement('x');
    xX.appendChild(xRoot.createTextNode(v2s(nodes(i).x)));
    xNode.appendChild(xX);
    
    xMesh.appendChild(xNode);
end
for k = 1:numel(springs)
    xSpring = xRoot.createElement('spring');
    
    % Spring attributes
    xSpring.setAttribute('idx', num2str(k));
    xSpring.setAttribute('k', num2str(springs(k).k));
    xSpring.setAttribute('m', num2str(springs(k).m));
    
    % Endpoint node indices
    xNode1 = xRoot.createElement('node1');
    xNode1.appendChild(xRoot.createTextNode(num2str(springs(k).nodes(1))));
    xSpring.appendChild(xNode1);
    
    xNode2 = xRoot.createElement('node2');
    xNode2.appendChild(xRoot.createTextNode(num2str(springs(k).nodes(2))));
    xSpring.appendChild(xNode2);
    
    % Rest configuration
    xRij0 = xRoot.createElement('rij0');
    xRij0.appendChild(xRoot.createTextNode(v2s(springs(k).rij0)));
    xSpring.appendChild(xRij0);
    
    xRji0 = xRoot.createElement('rji0');
    xRji0.appendChild(xRoot.createTextNode(v2s(springs(k).rji0)));
    xSpring.appendChild(xRji0);
    
    xL0 = xRoot.createElement('l0');
    xL0.appendChild(xRoot.createTextNode(v2s(springs(k).l0)));
    xSpring.appendChild(xL0);
    
    xMesh.appendChild(xSpring);
end
for i = 1:numel(groups)
    xGroup = xRoot.createElement('group');
    
    % Group attributes
    xGroup.setAttribute('idx', num2str(i));
    
    % Group actuators
    xX = xRoot.createElement('springs');
    xX.appendChild(xRoot.createTextNode(v2s(groups(i).springs)));
    xGroup.appendChild(xX);
    
    xMesh.appendChild(xGroup);
end

xmlwrite(filename, xRoot);
end

function st = v2s(vec)
% Vector 2 string
st = arrayfun(@num2str, vec, 'uni', false);
st = strjoin(st, ' ');
end