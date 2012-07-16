% Function to save Matrices from anim object as opencv yaml

function [ ] = anim2cvyaml(filename, anim )

f=fopen(filename,'w');
fprintf(f,'%s\n','%YAML:1.0');

%Check projection errors
for m=1:size(anim.P,3)  % views
    for n=1: size(anim.S,2) %  points
        x=anim.P(:,:,m)* [anim.S(:,n);1];
        x=x/x(3);
        x=x(1:2);
        e=x-anim.W(:,n,m);
        display(m);
        display(n);
        assert(e(1)==0);
        assert(e(2)==0);
    end
end


% Save 3D pts S
addArray(['S'],anim.S,f);

% save each frame
for n=1:anim.nFrame
    
    % W
    addArray(['W' num2str(n)],anim.W(:,:,n),f);
    
    % P
    addArray(['P' num2str(n)],anim.P(:,:,n),f);
    
    % R
    addArray(['R' num2str(n)],anim.R(:,:,n),f);
    
    % t
    addArray(['t' num2str(n)],anim.t(:,n),f);
    
end

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

a=reshape(array',1,size(array,1)*size(array,2));

str=['[ ' num2str(a(1))];
for i=2:length(a)
    str=[str ', ' num2str(a(i),8)];
end
str=[str ' ]'];

end


