MODEL_TEMPLATE_DIR ?= ../model_templates
AUTHOR_NAME ?= 'Gökçe Aydos'
AUTHOR_EMAIL ?= goekce_aydos_de
MODEL_DESC ?= ''

############################################################

vpath %.config.template $(MODEL_TEMPLATE_DIR)
vpath %.sdf             $(MODEL_TEMPLATE_DIR)

all: model.config model.sdf

MODEL_NAME := $(notdir $(shell pwd))
PARENT_CLASS := $(firstword $(subst _, ,$(MODEL_NAME)))
$(info $(PARENT_CLASS))
# First strip parent class name, then underscore. Otherwise stripping does not
# work for model names without any serialized arguments. For example 'wall' has
# no serialized arguments. Example: `_1_1` => `1_1`
PARENT_CLASS_ARGS := $(patsubst _%,%, \
	$(subst $(PARENT_CLASS),,$(MODEL_NAME)) \
	)

MAKO_RENDER_OPT := \
	--var model_name=$(MODEL_NAME)

# mako-render script does not work with relative dirs which are outside of the
# directory where mako-render runs. Use absolute path as a workaround using
# $(abspath ...)
model.config: model.config.template
	mako-render \
		--var author_name=$(AUTHOR_NAME) \
		--var author_email=$(AUTHOR_EMAIL) \
		--var model_desc=$(MODEL_DESC) \
		$(MAKO_RENDER_OPT) \
		$(abspath $<) \
		--output-file $@

model.sdf: $(PARENT_CLASS).sdf
	mako-render \
		$(MAKO_RENDER_OPT) \
		--var serialized_args=$(PARENT_CLASS_ARGS) \
		$(abspath $<) \
		--output-file $@

clean:
	$(RM) model.{config,sdf}
