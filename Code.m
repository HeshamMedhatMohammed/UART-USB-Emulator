clc;clear;
% decode the configuration.json file
config_file = fileread('conf.json');
json_structure = jsondecode(config_file);
[protocol_name1,protocol_name2] = json_structure.protocol_name;
[parameters1,parameters2] = json_structure.parameters;

% Any other protocol name result in an error message to the user
if ((protocol_name1 ~= "USB") && (protocol_name1 ~= "UART"))
    fprintf("ERROR MESSAGE: The First Protocol %s is not available\n",protocol_name1);
end
if ((protocol_name2 ~= "USB") && (protocol_name2 ~= "UART"))
    fprintf("ERROR MESSAGE: The Second Protocol %s is not available\n",protocol_name2);
end

% decode the information from the provided configuration file
if (protocol_name1 == "UART")
    data_bits = parameters1.data_bits;
    stop_bits = parameters1.stop_bits;
    parity = parameters1.parity;
    bit_duration_UART = parameters1.bit_duration;
end
if (protocol_name2 == "UART")
    data_bits = parameters2.data_bits;
    stop_bits = parameters2.stop_bits;
    parity = parameters2.parity;
    bit_duration_UART = parameters2.bit_duration;
end
if (protocol_name1 == "USB")
    sync_pattern = parameters1.sync_pattern;
    pid = parameters1.pid;
    dest_address = parameters1.dest_address;
    payload = parameters1.payload;
    bit_duration_USB = parameters1.bit_duration;
end
if (protocol_name2 == "USB")
    sync_pattern = parameters2.sync_pattern;
    pid = parameters2.pid;
    dest_address = parameters2.dest_address;
    payload = parameters2.payload;
    bit_duration_USB = parameters2.bit_duration;
end

% Turn the data in the input.txt to binary
fileID = fopen('input.txt');
data = fread(fileID);
Original_data = data;
fclose(fileID);

% Loop to change the file size
for new_byte=0:floor(length(data)/2)
    
    if (new_byte == 0)
        Bi_data = de2bi(data);
        b = (Bi_data)';
    else
        id = fopen('input.txt','a+');          %open the file and write more bytes
        for new=(2*new_byte-1):(2*new_byte)
            fprintf(id,char(data(new)));       %write two bytes at once
        end
        fclose(id);
        fileID = fopen('input.txt');
        data = fread(fileID);
        fclose(fileID);
        Bi_data = de2bi(data);
        b = (Bi_data)';
    end
    
    % start the code of the UART
    if ((protocol_name1 == "UART") || (protocol_name2 == "UART"))
        
        % code handle the change in the file size
        data_packets = floor((length(data)*7)/data_bits);
        extra_bits = rem((length(data)*7),data_bits);
        extra_packet = 0;
        b_UART = b;
        if (extra_bits ~= 0)
            needed_bits = data_bits - extra_bits;
            Z = zeros(needed_bits,1);
            temp = reshape(b_UART,[],1);
            b_UART = [temp;Z];
            extra_packet = 1;
        end
        Binary_data = reshape(b_UART,data_bits,(data_packets + extra_packet));
        
        % Code responsible for parity bit
        counter_vector = zeros(1,(data_packets + extra_packet));
        parity_vector = zeros(1,(data_packets + extra_packet));
        parity_bit = 0;
        if (parity == "even")
            parity_bit = 1;
            for j=1:(data_packets + extra_packet)
                for i=1:data_bits
                    if (Binary_data(i,j)==1)
                        counter_vector(1,j) = counter_vector(1,j)+1;
                    end
                end
            end
            for j = 1:(data_packets + extra_packet)
                if (rem(counter_vector(1,j),2))
                    parity_vector(1,j) = 1;
                end
                binary_frame = [Binary_data;parity_vector];
            end
        elseif (parity == "odd")
            parity_bit = 1;
            for j=1:(data_packets + extra_packet)
                for i=1:data_bits
                    if (Binary_data(i,j)==1)
                        counter_vector(1,j) = counter_vector(1,j)+1;
                    end
                end
            end
            for j = 1:(data_packets + extra_packet)
                if (~(rem(counter_vector(1,j),2)))
                    parity_vector(1,j) = 1;
                end
                binary_frame = [Binary_data;parity_vector];
            end
        else
            binary_frame = Binary_data;
        end
        
        % Preparing the start and stop bit of each packet.
        startbit_vector = zeros(1,(data_packets + extra_packet));
        stopbit_vector = ones(stop_bits,(data_packets + extra_packet));
        binary_frame = [startbit_vector;binary_frame;stopbit_vector];
        
        % the final binary to be plotted
        binary_stream = reshape(binary_frame,[],1);
        X = (1:2*(1+data_bits+parity_bit+stop_bits)+1);
        
        if (new_byte == 0)
            
            % ploting
            figure
            stairs(X,binary_stream(1:length(X)));
            title('UART')
            xlim([1 length(X)])
            ylim([-1 2])
            transmition_time_UART = bit_duration_UART * length(binary_stream);
            bin_data = reshape(b_UART,[],1);
            efficiency_UART = (length(bin_data)-extra_bits)/(length(binary_stream)+extra_bits);
            overhead_UART = 1-efficiency_UART;
            
            transmition_time_UART_vector = zeros(1,floor(length(data)/2));  %(data_packets + extra_packet)
            efficiency_UART_vector = zeros(1,floor(length(data)/2));
            overhead_UART_vector = zeros(1,floor(length(data)/2));
            
        else
            % calculating the efficiency, transmition_time and overhead to be
            % plotted
            transmition_time_UART_vector(new_byte) = bit_duration_UART * length(binary_stream);
            bin_data = reshape(b_UART,[],1);
            efficiency_UART_vector(new_byte) = (length(bin_data)-extra_bits)/(length(binary_stream)+extra_bits);
            overhead_UART_vector(new_byte) = 1-efficiency_UART_vector(new_byte);
        end
    end
    
    % start the code of USB
    if ((protocol_name1 == "USB") || (protocol_name2 == "USB"))
        s = dir('input.txt');
        filesize = s.bytes;
        Bi_data = de2bi(data);
        b_USB = (Bi_data)';
        y=reshape(b_USB,[],1);
        no_bits_per_packet=payload.*8;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        actual_byte=((filesize.*7)/(128.*8));
        no_packets= ceil(actual_byte);
        no_PID=rem(1:no_packets,16);
        no_PID=(no_PID)';
        Bi_PID = de2bi(no_PID);
        Bi_PID=(Bi_PID)';
        Bi_PID_complement=~Bi_PID;
        concatenate_Bi_PID=vertcat(Bi_PID ,Bi_PID_complement);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %no.bytes that not be perform 128 byte in packet
        x=rem((filesize.*7),(128.*8));
        %bytes that not be perform 128 byte in packet
        Bi_remainder_of_data=b_USB(end-x+1:end);
        
        if (x==0)
            data_packet_USB=reshape(b_USB,no_bits_per_packet,((filesize*7)/(128*8)));
        else
            Bi_remainder_of_data_transpose_reshape=reshape(Bi_remainder_of_data,[],1);
            E_packet=y([1:((filesize.*7-x))],1);
            E_datapacket=reshape(E_packet,[],(no_packets-1));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %sync_pattern conversion from ASCI >> DECIMAL
        sync_pattern_USB=sync_pattern-48;
        sync_pattern_USB=(sync_pattern_USB)';
        repeated_sync_pattern=repmat(sync_pattern_USB,1,no_packets);
        
        %data_address conversion from ASCI >> DECIMAL
        dest_address_USB=dest_address-48;
        dest_address_USB = fliplr(dest_address_USB);
        dest_address_USB=(dest_address_USB)';
        repeated_dest_address=repmat(dest_address_USB,1,no_packets);
        concatenate_sync_pattern_Bi_PID=vertcat(repeated_sync_pattern , concatenate_Bi_PID);
        concatenate_sync_pattern_Bi_PID_dest_address=vertcat(concatenate_sync_pattern_Bi_PID,repeated_dest_address);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% << stopbits >> %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        EOP_bits=[5;5;3];
        repeated_dstop_bits=repmat(EOP_bits,1,no_packets);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% << concatenate with data >>%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (rem((filesize.*7),(128.*8))==0)
            concatenate_sync_pattern_Bi_PID_dest_address_Data=vertcat(concatenate_sync_pattern_Bi_PID_dest_address,data_packet_USB,repeated_dstop_bits);
            reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data=reshape(concatenate_sync_pattern_Bi_PID_dest_address_Data,[],1);
        else
            concatenate_sync_pattern_Bi_PID_dest_address_Data=vertcat(concatenate_sync_pattern_Bi_PID_dest_address(:,[1:(no_packets-1)]), E_datapacket,repeated_dstop_bits(:,[1:(no_packets-1)]));
            last_packet=vertcat(concatenate_sync_pattern_Bi_PID_dest_address(:,no_packets),Bi_remainder_of_data_transpose_reshape,repeated_dstop_bits(:,no_packets));
            reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data=reshape(concatenate_sync_pattern_Bi_PID_dest_address_Data,[],1);
            reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data=vertcat(reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data,last_packet);
        end
        
        %%%%%%%%%%%%%%%% bit stuffing%%%%%%%%%%%%%%%%%%
        msg=(reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data)';
        count=0;
        stuffcount=0;
        
        [M N]=size(msg);
        for j=1:N-6+stuffcount
            for i=j:j+5
                if msg(i)==1
                    count=count+1;
                else
                    count=0;
                    break;
                end
            end
            if(count ==6)
                msg=[msg(1:j+5) 0 msg(j+6 : end)];
                count=0;
                stuffcount=stuffcount+1;
            end
        end
        out_stuff=(msg)';
        
        %%%%%%%%%%%%%%%%%%%%%%% << NRZI >> %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        NRZI= zeros(1,length(out_stuff));
        NRZI_Inverted= zeros(1,length(out_stuff));
        NRZI(1)=0;
        NRZI_Inverted(1)=1;
        for i=2:length(out_stuff)
            if out_stuff(i)==0
                NRZI(i)=~(NRZI(i-1));
                NRZI_Inverted(i)=~(NRZI(i));
            elseif out_stuff(i)==1
                NRZI(i)=(NRZI(i-1));
                NRZI_Inverted(i)=~(NRZI(i));
            elseif out_stuff(i)==3
                NRZI(i)=1;
                NRZI_Inverted(i)=~(NRZI(i));
            elseif out_stuff(i)==5
                NRZI(i)=5;
                NRZI_Inverted(i)=(NRZI(i));
            end
        end
        for i=1:length(NRZI_Inverted)
            if NRZI_Inverted(i)==5
                NRZI_Inverted(i)=0;
                NRZI(i)=0;
            end
            if NRZI_Inverted(i)==3
                NRZI_Inverted(i)=0;
                NRZI(i)=1;
            end
        end
        X_USB = (1:2*(length(sync_pattern_USB)+pid+length(dest_address_USB)+payload*8+stuffcount+2)+1);
        
        if (new_byte == 0)
            
            % plotting
            figure
            subplot(2,1,1);
            stairs(X_USB,NRZI(1:length(X_USB)))
            title('USB D+')
            xlim([1 length(X_USB)])
            %xlim([17 45])
            ylim([-1 2])
            
            subplot(2,1,2);
            stairs(X_USB,NRZI_Inverted(1:length(X_USB)))
            title('USB D-')
            xlim([1 length(X_USB)])
            %xlim([17 45])
            ylim([-1 2])
            
            transmition_time_USB = bit_duration_USB * length(NRZI);
            efficiency_USB = (filesize.*7)/(length(NRZI));
            overhead_USB = 1-efficiency_USB;
            
            transmition_time_USB_vector = zeros(1,floor(length(data)/2));
            efficiency_USB_vector = zeros(1,floor(length(data)/2));
            overhead_USB_vector = zeros(1,floor(length(data)/2));
            transmition_time_USB_vector_w = zeros(1,floor(length(data)/2));
            efficiency_USB_vector_w = zeros(1,floor(length(data)/2));
            overhead_USB_vector_w = zeros(1,floor(length(data)/2));
        else
            
            % calculating the efficiency, transmition_time and overhead to be
            % plotted
            transmition_time_USB_vector(new_byte) = bit_duration_USB * length(NRZI);
            efficiency_USB_vector(new_byte) = (filesize.*7)/(length(NRZI));
            overhead_USB_vector(new_byte) = 1-efficiency_USB_vector(new_byte);
            transmition_time_USB_vector_w(new_byte) = bit_duration_USB * length((reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data)');
            efficiency_USB_vector_w(new_byte) = (filesize.*7)/(length((reshape_concatenate_sync_pattern_Bi_PID_dest_address_Data)'));
            overhead_USB_vector_w(new_byte) = 1-efficiency_USB_vector_w(new_byte);
        end
    end
end

% return the input file to its original case
id = fopen('input.txt','w');
fprintf(id,char(Original_data));
fclose(id);

if (~((protocol_name1 ~= "USB") && (protocol_name1 ~= "UART")))
    if (~((protocol_name2 ~= "USB") && (protocol_name2 ~= "UART")))
        if (bit_duration_UART == bit_duration_USB )
            
            % printing the efficiency, transmition_time and overhead in output.json
            output_file = fileread('output.json');
            output_json_structure = jsondecode(output_file);
            output_json_structure(1).outputs.total_tx_time = transmition_time_UART;
            output_json_structure(1).outputs.overhead =  overhead_UART;
            output_json_structure(1).outputs.efficiency = efficiency_UART;
            output_UART = jsonencode(output_json_structure);
            P_output_json_structure_UART = prettyjson(output_UART);
            fid = fopen('output.json','w');
            fprintf(fid,'%s',P_output_json_structure_UART);
            fclose(fid);
            
            output_file = fileread('output.json');
            output_json_structure = jsondecode(output_file);
            output_json_structure(2).outputs.total_tx_time = transmition_time_USB;
            output_json_structure(2).outputs.overhead =  overhead_USB;
            output_json_structure(2).outputs.efficiency = efficiency_USB;
            output_USB = jsonencode(output_json_structure);
            P_output_json_structure_USB = prettyjson(output_USB);
            fid = fopen('output.json','w');
            fprintf(fid,'%s',P_output_json_structure_USB);
            fclose(fid);
        end
    end
    
    % plotting the the efficiency, transmition_time and overhead VS change
    % in file size
    figure
    plot([overhead_UART,overhead_UART_vector])
    title('UART Overhead VS file size')
    xlabel('Number of added words (2bytes) to the original file size')
    xlim([0 64])
    %ylim([0.1999 0.201])
    figure
    plot([transmition_time_UART,transmition_time_UART_vector])
    title('UART transmition time VS file size')
    xlabel('Number of added words (2bytes) to the original file size')
    ylabel('Seconds')
    figure
    plot([efficiency_UART,efficiency_UART_vector])
    title('UART efficiency VS file size')
    xlabel('Number of added words (2bytes) to the original file size')
    xlim([0 64])
    %ylim([0.799 0.8001])
    
    figure
    subplot(2,1,1)
    plot(overhead_USB_vector_w)
    title('USB Overhead VS file size (without stuffed bits)')
    xlabel('Number of added words (2bytes) to the original file size')
    subplot(2,1,2)
    plot([overhead_USB,overhead_USB_vector])
    title('USB Overhead VS file size (with stuffed bits)')
    xlabel('Number of added words (2bytes) to the original file size')
    %ylim([0.1997 0.2015])
    figure
    plot([transmition_time_USB,transmition_time_USB_vector])
    title('USB transmition time VS file size')
    xlabel('Number of added words (2bytes) to the original file size')
    ylabel('Seconds')
    %ylim([0.798 0.801])
    figure
    subplot(2,1,1)
    plot(efficiency_USB_vector_w)
    title('USB efficiency VS file size (without stuffed bits)')
    xlabel('Number of added words (2bytes) to the original file size')
    subplot(2,1,2)
    plot([efficiency_USB,efficiency_USB_vector])
    title('USB efficiency VS file size (with stuffed bits)')
    xlabel('Number of added words (2bytes) to the original file size')
    %ylim([0.798 0.801])
end

% function to pretty the output text in output.json file
function [less_ugly] = prettyjson(ugly)
% Makes JSON strings (relatively) pretty
% Probably inefficient

% Mostly meant for structures with simple strings and arrays;
% gets confused and !!mangles!! JSON when strings contain [ ] { or }.

MAX_ARRAY_WIDTH = 80;
TAB = '    ';

ugly = strrep(ugly, '{', sprintf('{\n'));
ugly = strrep(ugly, '}', sprintf('\n}'));
ugly = strrep(ugly, ',"', sprintf(', \n"'));
ugly = strrep(ugly, ',{', sprintf(', \n{'));

indent = 0;
lines = splitlines(ugly);

for i = 1:length(lines)
    line = lines{i};
    next_indent = 0;
    
    % Count brackets
    open_brackets = length(strfind(line, '['));
    close_brackets = length(strfind(line, ']'));
    
    open_braces = length(strfind(line, '{'));
    close_braces = length(strfind(line, '}'));
    
    if close_brackets > open_brackets || close_braces > open_braces
        indent = indent - 1;
    end
    
    if open_brackets > close_brackets
        line = strrep(line, '[', sprintf('[\n'));
        next_indent = 1;
    elseif open_brackets < close_brackets
        line = strrep(line, ']', sprintf('\n]'));
        next_indent = -1;
    elseif open_brackets == close_brackets && length(line) > MAX_ARRAY_WIDTH
        first_close_bracket = strfind(line, ']');
        if first_close_bracket > MAX_ARRAY_WIDTH % Just a long array -> each element on a new line
            line = strrep(line, '[', sprintf('[\n%s', TAB));
            line = strrep(line, ']', sprintf('\n]'));
            line = strrep(line, ',', sprintf(', \n%s', TAB)); % Add indents!
        else % Nested array, probably 2d, first level is not too wide -> each sub-array on a new line
            line = strrep(line, '[[', sprintf('[\n%s[', TAB));
            line = strrep(line, '],', sprintf('], \n%s', TAB)); % Add indents!
            line = strrep(line, ']]', sprintf(']\n]'));
        end
    end
    
    sublines = splitlines(line);
    for j = 1:length(sublines)
        if j > 1   % todo: dumb to do this check at every line...
            sublines{j} = sprintf('%s%s', repmat(TAB, 1, indent+next_indent), sublines{j});
        else
            sublines{j} = sprintf('%s%s', repmat(TAB, 1, indent), sublines{j});
        end
    end
    
    if open_brackets > close_brackets || open_braces > close_braces
        indent = indent + 1;
    end
    indent = indent + next_indent;
    lines{i} = strjoin(sublines, newline);
    
end

less_ugly = strjoin(lines, newline);
end