function label=label2mat(bin_path)
fid = fopen(bin_path, 'rb');
label=fread(fid, [1 inf], 'uint'); 
fclose(fid);
end