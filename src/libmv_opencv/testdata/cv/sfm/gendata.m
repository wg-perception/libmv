% Create test data using:
%
% @misc{vincentsSFMToolbox,
%  Author = {Vincent Rabaud},
%  Title = {Vincent's {S}tructure from {M}otion {T}oolbox},
%  howpublished = {\url{http://vision.ucsd.edu/~vrabaud/toolbox/}}
% }
% The previous home for that project was on http://vision.ucsd.edu/~vrabaud/toolbox/doc/
% 

% anim=generateToyAnimation( 0,'nPoint',5,'nFrame',3);
% anim2cvyaml('rnd_N5_F3.yml',anim);

anim=generateToyAnimation( 0,'nPoint',10,'nFrame',3,'isProj',1);
anim2cvyaml('rnd_N10_F3_Proj1.yml',anim);

% anim=generateToyAnimation( 0,'nPoint',10,'nFrame',3,'isProj',0);
% anim2cvyaml('rnd_N10_F3_Proj0.yml',anim);
