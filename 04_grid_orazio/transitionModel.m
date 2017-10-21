function transition_probability_matrix = transitionModel(map_, row_from_, col_from_, control_input_)
  map_rows = rows(map_);
  map_cols = columns(map_);
  transition_probability_matrix = zeros(map_rows, map_cols);

  #against each other cell and itself
	for row_to = 1:map_rows
		for col_to = 1:map_cols
	
      #available robot controls (corresponding to keyboard key values)
      global MOVE_UP;
      global MOVE_DOWN;
      global MOVE_LEFT;
      global MOVE_RIGHT;

      #compute resulting position difference
      translation_rows = row_to - row_from_;
      translation_cols = col_to - col_from_;

      #allow only unit motions (1 cell): check if we have a bigger motion
      if(abs(translation_rows) > 1 || abs(translation_cols) > 1)
	      continue;
      endif

      #compute target robot position according to input
      target_row = row_from_;
      target_col = col_from_;
      switch (control_input_)
	      case MOVE_UP
		      target_row--;
	      case MOVE_DOWN
		      target_row++;
	      case MOVE_LEFT 
		      target_col--;
	      case MOVE_RIGHT 
		      target_col++;
	      otherwise
		      return;
      endswitch

	    #check if the desired motion is infeasible
	    invalid_motion = false;
	    if (target_row < 1 || target_row > map_rows || target_col < 1 || target_col > map_cols) #if we're going over the border
		    invalid_motion = true;
	    elseif (map_(target_row, target_col) == 1 || map_(row_to, col_to) == 1) #obstacle in the goal cell
		    invalid_motion = true;
	    endif
	    if (invalid_motion)
	
	      #if the desired translation is zero
	      if (translation_rows == 0 && translation_cols == 0)
          transition_probability_matrix(row_to, col_to) = 1; #we stay with 100% probability (no motion has full confidence)
		      continue;
	      else
	        continue; #we cannot move
	      endif
	    endif

      #our motion is feasible - compute resulting transition
      switch (control_input_)
        case MOVE_UP 
          if (translation_rows     == -1 && translation_cols ==  0) transition_probability_matrix(row_to, col_to) = 0.8;
          elseif (translation_rows == -1 && translation_cols ==  1) transition_probability_matrix(row_to, col_to) = 0.1;
          elseif (translation_rows == -1 && translation_cols == -1) transition_probability_matrix(row_to, col_to) = 0.1;
          endif;
        case MOVE_DOWN 
          if (translation_rows     ==  1 && translation_cols ==  0) transition_probability_matrix(row_to, col_to) = 0.8;
          elseif (translation_rows ==  1 && translation_cols ==  1) transition_probability_matrix(row_to, col_to) = 0.1;
          elseif (translation_rows ==  1 && translation_cols == -1) transition_probability_matrix(row_to, col_to) = 0.1;
		      endif;	
        case MOVE_LEFT
          if (translation_rows     ==  0 && translation_cols == -1) transition_probability_matrix(row_to, col_to) = 0.8;
          elseif (translation_rows ==  1 && translation_cols == -1) transition_probability_matrix(row_to, col_to) = 0.1;
          elseif (translation_rows == -1 && translation_cols == -1) transition_probability_matrix(row_to, col_to) = 0.1;
          endif;
        case MOVE_RIGHT
          if (translation_rows     ==  0 && translation_cols ==  1) transition_probability_matrix(row_to, col_to) = 0.8;
          elseif (translation_rows ==  1 && translation_cols ==  1) transition_probability_matrix(row_to, col_to) = 0.1;
          elseif (translation_rows == -1 && translation_cols ==  1) transition_probability_matrix(row_to, col_to) = 0.1;
          endif;
      endswitch
    endfor
  endfor
endfunction

