

%{

  //Disc Model
  //contains the name and the physics/aerodynamic properties of a disc mold
  struct Disc_Model
  {
    char mold_name[32];     // text for disc name, e.g. Buzzz Big Z
    char manufacturer[32];  // e.g. innova (could do this with an enum later)
    char disc_type[32];     // e.g. putter, midrange, distance driver, etc. (could do this with an enum later)
    float mass;             // (kg), this is just a default can be changed later as part of user config
    float radius;           // (m)
    float rim_width;        // width of rim, from edge of cavity, to edge of disc (m)
    float thickness;        // total thickness of disc, from bottom of rim, to top of camber (m)
    float rim_depth;        // height of cavity, measured at the edge of the cavity (m)
    float edge_height;      // approximation of 'edge height' for our simpified disc model (m)    
  };
%}

current_dir = '~/sandbox/disc_mold_csvs/'
files=dir([current_dir, '*.csv']);
for i=1:length(files)
   file_names{i}=files(i).name;
end

out_filename = [current_dir, 'disc_params.hpp_innards'];

fileID = fopen(out_filename,'w');

count = 0;

for i=1:length(files)
  
  split_filename = strsplit(char(file_names{i}),' ');
  split_filename = strsplit(split_filename{end},'.');
  manufacturer   = split_filename{1};
  
  M = readtable([current_dir, file_names{i}]);
  
  for k = 1:length(M{:, 1})
 
    mold_name     = char(M{k, 1});
    disc_type     = char(M{k, 3});
    plastic       = char(M{k, 4});
    stability     = char(M{k, 2});
    mass          = double((M{k, 5}))  * 0.001;
    radius        = double((M{k, 6}))  * 0.001 * 0.5; % halve diameter
    rim_width     = double((M{k, 7}))  * 0.001;
    thickness     = double((M{k, 8}))  * 0.001;
    rim_depth     = double((M{k, 9}))  * 0.001;
    edge_height   = double((M{k, 10})) * 0.001;
    
    fprintf(fileID,'    // %d\n', count);
    fprintf(fileID,'    Disc_Model\n');
    fprintf(fileID,'    { // %s %s (%s)\n', manufacturer, mold_name, plastic);
    fprintf(fileID,'      /*.mold_name =*/     "%s",\n',    mold_name);
    fprintf(fileID,'      /*.manufacturer =*/  "%s",\n',    manufacturer);
    fprintf(fileID,'      /*.disc_type =*/     "%s",\n',    disc_type);
    fprintf(fileID,'      /*.stability =*/     "%s",\n',    stability);
    fprintf(fileID,'      /*.mass =*/          %0.4f,\n', mass);
    fprintf(fileID,'      /*.radius =*/        %0.4f,\n', radius);
    fprintf(fileID,'      /*.rim_width =*/     %0.4f,\n', rim_width);
    fprintf(fileID,'      /*.thickness =*/     %0.4f,\n', thickness);
    fprintf(fileID,'      /*.rim_depth =*/     %0.4f,\n', rim_depth);
    fprintf(fileID,'      /*.edge_height =*/   %0.4f\n', edge_height);
    fprintf(fileID,'    },\n');

    disp(sprintf('Wrote %s %s (%s) entry', manufacturer, mold_name, plastic));
    count = count + 1;
    
  end
  
end


fclose(fileID);

 %{
   // 0
    Disc_Model
    {  // make the 'NONE' one a brick
      /*.mold_name =*/     "Mr. Brick",
      /*.manufacturer =*/  "Innova",
      /*.disc_type =*/     "Putter",
      /*.mass =*/           
      /*.radius =*/
      /*.rim_width =*/
      /*.thickness =*/
      /*.rim_depth =*/
      /*.edge_height =*/
    },
%}
