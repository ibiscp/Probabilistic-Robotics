#clear octave
close all
clear
clc

#check input
if length(argv) == 0
  printf("no input map provided - run: octave grid_orazio maps/map.txt\n");
  return;
endif

#generate/load our map
global map = getMap(argv(){1});

#initialize actual robot position x,y (not a state visible to the robot)
global state_ground_truth = [rows(map)-1, 2];

#initialize robot states: position belief values over the complete grid
number_of_free_cells = rows(map)*columns(map);
belief_initial_value = 1/(number_of_free_cells);
global state_belief = ones(rows(map), columns(map))*belief_initial_value;
global observations  = [0 0 0 0];

#function that is executed when a key is pressed (user input)
function keyPressed (src_, event_)

  #available robot controls (corresponding to keyboard key values)
  global MOVE_UP    = 119; # W
  global MOVE_DOWN  = 115; # S
  global MOVE_LEFT  = 97;  # A
  global MOVE_RIGHT = 100; # D
  
  #evaluate which key was pressed
  control_input = 0;
  switch(event_.Key)
    case MOVE_UP
      control_input = MOVE_UP;
      control_input_string = "MOVE UP";
    case MOVE_DOWN
      control_input = MOVE_DOWN;
      control_input_string = "MOVE DOWN";
    case MOVE_LEFT
      control_input = MOVE_LEFT;
      control_input_string = "MOVE LEFT";
    case MOVE_RIGHT
      control_input = MOVE_RIGHT;
      control_input_string = "MOVE RIGHT";
    otherwise
      return;
  endswitch
  fflush(stdout);
  
  #fetch global variables
  global map;
  global state_ground_truth;
  global state_belief;
  global observations;
  global subfigure_ground_truth;
  global subfigure_belief;
  map_rows = rows(map);
  map_cols = columns(map);
  
  #erase previous robot position and observations
  subplot(subfigure_ground_truth);
  drawRectangle(map, state_ground_truth(1), state_ground_truth(2), "white");
  clearObservations(observations, state_ground_truth(1), state_ground_truth(2), map);
  
#---------------------------------- FILTERING ----------------------------------

  #retrieve new robot position according to our transition model
  state_ground_truth = getNextState(map, state_ground_truth, control_input);
  
  #obtain current observations according to our observation model
  observations = getObservations(map, state_ground_truth(1), state_ground_truth(2));

  #PREDICT robot position belief
  state_belief_previous = state_belief;
  state_belief = zeros(map_rows, map_cols);
	for row = 1:map_rows
		for col = 1:map_cols
      state_belief += transitionModel(map, row, col, control_input)*state_belief_previous(row, col);
		endfor
	endfor

  #UPDATE robot position belief and COMPUTE the normalizer
	inverse_normalizer = 0;
	for row = 1:map_rows
		for col = 1:map_cols
			state_belief(row, col) *= observationModel(map, row, col, observations);
      inverse_normalizer     += state_belief(row, col);
		endfor
	endfor	

  #NORMALIZE the belief probabilities to [0, 1]
	normalizer = 1./inverse_normalizer;	
	state_belief *= normalizer;

#---------------------------------- FILTERING ----------------------------------

  #draw new robot position and its observations on the map
  subplot(subfigure_ground_truth);
  drawRectangle(map, state_ground_truth(1), state_ground_truth(2), "red");
  drawObservations(map, observations, state_ground_truth(1), state_ground_truth(2));

  #draw belief
  subplot(subfigure_belief);
  drawBelief(state_belief, map);

  #status info
  printf("control input: %s, current position: %i %i, observations: %i %i %i %i\n", 
         control_input_string, state_ground_truth(1), state_ground_truth(2),
         observations(1), observations(2), observations(3), observations(4));
endfunction

#function that is executed once the GUI window is closed
function closedGUI (src_, data_)
  printf("GUI closed, terminating\n");
  global is_gui_active;
  is_gui_active = false;
endfunction

#create a figure and hook our callback functions to it
figure("name", "grid_orazio",      #figure title
       "deletefcn", @closedGUI,    #function that is called when the GUI is closed
       "keypressfcn", @keyPressed, #function that is called when a key is pressed
       "selected", "on",           #put figure into focus
       "numbertitle", "off");      #remove figure number
font_size = 15;

#initialize robot position belief
global subfigure_belief = subplot(1, 2, 1);
grid on;
axis("square");
set(title("robot position | belief"), "fontsize", font_size);
drawBelief(state_belief, map);

#initialize robot position
global subfigure_ground_truth = subplot(1, 2, 2);
grid on;
axis("square");
set(title("robot position | ground truth"), "fontsize", font_size);
drawMap(map);

#draw initial robot position
drawRectangle(map, state_ground_truth(1), state_ground_truth(2), "red");
printf("map loaded!\n");
printf("Move orazio with [W, A, S, D]\n");

#display GUI as long as active
global is_gui_active = true;
while(is_gui_active)
  refresh();  #refresh figures (callback functions are triggered if keys are pressed)
  pause(0.1); #relax the cpu
endwhile
printf("terminated\n");

#clear octave
close all
clear
clc

