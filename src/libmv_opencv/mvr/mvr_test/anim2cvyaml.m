function [ ] = anim2cvyaml(filename, anim )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

f=fopen(filename,'w');
fprintf(f,'%s\n','%YAML:1.0');

addArray('P1',anim.P(:,:,1),f)

fclose(f);

end

%%
function []=addArray(name,array,fp)

fprintf(fp,'%s: !!opencv-matrix\n', name);
fprintf(fp,'   rows: %d\n', size(array,1));
fprintf(fp,'   cols: %d\n', size(array,2));
fprintf(fp,'   dt: d\n');
fprintf(fp,'   data: %s\n', mat2datastr(array));

end

%%

function [str]=mat2datastr(array)

a=reshape(array,1,size(array,1)*size(array,2));

str=['[ ' num2str(a(1))];
for i=2:length(a)
    str=[str ', ' num2str(a(i))];
end
str=[str ' ]'];

end


