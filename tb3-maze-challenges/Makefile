MODEL_TEMPLATE_DIR ?= model_templates

#############################################################
vpath %.sdf $(MODEL_TEMPLATE_DIR)

all: \
	world_1_1.sdf \
	world_2_2.sdf \
	world_3_3.sdf \
	world_3_3_box.sdf \

competition: \
	world_3_3_a.sdf \
	world_3_3_b.sdf \
	world_3_3_c.sdf

LABYRINTH_COMMON_PREREQ_MODELS := \
	wall/ \
	wall_White/ \
	wall_Red/

world_%.sdf: challenge.sdf labyrinth_%/
	mako-render \
		--var world_name=$(basename $@) \
		--var labyrinth_name=$(subst /, ,$(word 2,$^)) \
		--var robot_pose_zr=pi/2 \
		$< > $@

world_3_3%.sdf: challenge.sdf labyrinth_3_3%/
	mako-render \
		--var world_name=$(basename $@) \
		--var labyrinth_name=$(subst /, ,$(word 2,$^)) \
		--var robot_pose_zr=pi/2 \
		$< > $@

world_3_3_box.sdf: challenge.sdf labyrinth_3_3_box/
	mako-render \
		--var world_name=$(basename $@) \
		--var labyrinth_name=$(subst /, ,$(word 2,$^)) \
		--var robot_pose_zr=pi/2 \
		$< > $@
	sed -i 's|\(</world>\)|$(CHOCOLATE_3_3)\n\1|' $@

CHOCOLATE_3_3 := <include><name>chocolate</name><uri>https://fuel.ignitionrobotics.org/1.0/GoogleResearch/models/KS_Chocolate_Cube_Box_Assortment_By_Neuhaus_2010_Ounces</uri><pose>0.5 2.5 0 0 0 0</pose></include>

labyrinth_1_1/ \
labyrinth_2_2/ \
labyrinth_3_3/ \
labyrinth_3_3_box/ \
labyrinth_3_3_a/ \
labyrinth_3_3_b/ \
labyrinth_3_3_c/ \
	: | $(LABYRINTH_COMMON_PREREQ_MODELS) maze/

maze/:
	# https://scipython.com/blog/making-a-maze/
	mkdir maze
	curl https://raw.githubusercontent.com/scipython/scipython-maths/master/maze/df_maze.py > $@/df_maze.py

# trigger makefiles of all models if one file in model_templates changes
%/: model_templates
	# Create $@ only if mako-render exists. Otherwise make does not remove $@ and
	# assumes that the model is made.
	@which mako-render && echo mako-render is not available
	mkdir -p $@
	ln -srf gazebo_model.mk $@/Makefile
	$(MAKE) -C $@

clean:
	$(RM) -r labyrinth_*
	$(RM) -r $(LABYRINTH_COMMON_PREREQ_MODELS)
	$(RM) world_*.sdf
	$(RM) -r maze
