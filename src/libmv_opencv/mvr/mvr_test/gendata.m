% Create test data using:
%
% @misc{vincentsSFMToolbox,
%  Author = {Vincent Rabaud},
%  Title = {Vincent's {S}tructure from {M}otion {T}oolbox},
%  howpublished = {\url{http://vision.ucsd.edu/~vrabaud/toolbox/}}
% }
% The previous home for that project was on http://vision.ucsd.edu/~vrabaud/toolbox/doc/

system(' rm -rf S* K* W* P*');


N=5;
F=3;

anim=generateToyAnimation( 0,'nPoint',N,'nFrame',F);

anim2cvyaml('anim.yml',anim);

% playAnim(anim);
% 
% S=anim.S;
% csvwrite('S',S);
% 
% Sh=[S;ones(1,N)];
% 
% 
% 
% for i=1:F
%        
%     W=anim.W(:,:,i);
%     P=anim.P(:,:,i);
%     K=eye(3,3);
%     
%     % Validate
%     Wh=K*P*Sh;
%     assert(norm(Wh(1:2,:)-W) == 0);
%     
%     
%     csvwrite(['W' int2str(i)],W);
%     csvwrite(['P' int2str(i)],P);
%     csvwrite(['K' int2str(i)],K);
%     
% end
