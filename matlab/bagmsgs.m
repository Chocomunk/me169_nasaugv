function  msgs = bagmsgs(bag, topicname)
%
%   msgs = bagmsgs(bag, topicname)
%
%   Extract the messages of the named topic from the given ROS bag.
%   The messages are returned as a struct array.  The structure
%   contains MessageType as well as the fields of the topic.
%

% Isolate the specified topic.
topic = select(bag, 'Topic', topicname);
if (~topic.NumMessages)
    warning(['No messages under topic ''' topicname '''']);
end

% Convert the messages in the topic into structure array.
msgs = cell2mat(readMessages(topic, 'DataFormat', 'struct'));

end
