function  msgs = rosbagmsgs(bagfilename, topicname)
%
%   msgs = rosbagmsgs(bagfilename, topicname)
%
%   Extract the messages of the named topic from the bagfile.  The
%   messages are returned as a struct array.  The structure contains
%   MessageType as well as the fields of the topic.
%

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end

% Grab the messages.
msgs = bagmsgs(bag, topicname);

end
