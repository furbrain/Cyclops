# Builds a model's assets from what the device records (atlas.osa, images.tar, and optionally survey.svx
# or survey.3d). Copy it into the model directory and run make there, or use
#   make -C <model dir> -f <path to this Makefile> <target>
# Targets can be file names or the asset names model.py uses (pointcloud, coarse, coarse_texture, ...).
# A combined model (one with a combine.json) has neither atlas.osa nor images.tar: its atlas.json.gz comes
# from combine.py, which needs each sub-survey's atlas (brought up to date by running this Makefile in the
# sub-survey's folder), and its images from its sub-surveys.

RASPBIAN_DIR=/home/pi/Cyclops/raspbian
# for running make in sub-surveys' folders
THIS_MAKEFILE := $(abspath $(lastword $(MAKEFILE_LIST)))

# a failed or interrupted step mustn't leave a partial output that looks up to date
.DELETE_ON_ERROR:

colmap_files = sparse/images.txt sparse/points3D.txt sparse/cameras.txt
pointcloud_files = pointcloud.json track.json info.json
# a survey can come as survey.svx (run through cavern) or just as survey.3d
SURVEY := $(if $(wildcard survey.svx survey.3d),survey.3d)
# sub-surveys, in a combined model: everything linked in combine.json
SUBS := $(shell python3 -c 'import json; print(*sorted({s for x in json.load(open("combine.json"))["links"] \
                                                       for s in (x["from"], x["to"])}))' 2>/dev/null)
SUB_ATLASES := $(SUBS:%=%/atlas.json.gz)
IMAGES := $(wildcard images.tar $(SUBS:%=%/images.tar))

# --- Asset names, as in model.py's ASSET_DEFS ---------------------------------

.PHONY: all info pointcloud track survey coarse coarse_texture hi-res hi-res_texture sparse

all: info coarse_full.glb hires_full.glb
# everything the viewer shows apart from the meshes
info: $(pointcloud_files) $(if $(SURVEY),survey.json)
pointcloud: pointcloud.json
track: track.json
survey: survey.json
coarse: coarse_mesh.ply
coarse_texture: coarse_full.glb
hi-res: hires_mesh.ply
hi-res_texture: hires_full.glb
# the sparse COLMAP model, as uploaded to FrogCore
sparse: $(colmap_files)

# --- Helpers ------------------------------------------------------------------

# Run $(1) with every images.tar this model uses mounted alongside it at images/ (ratarmount creates it):
# its own, or for a combined model each sub-survey's (found through the sub-survey symlinks, so sub-surveys
# that are combined models themselves work too). Then run the optional cleanup $(2), whether or not $(1)
# succeeded. An images/ that's already populated (extracted, or mounted by someone else) is used as it is.
define with_images
status=; mounted=; \
for tar in $$(find -L . -name images -prune -o -name images.tar -print); do \
  dir=$${tar%.tar}; \
  if [ -d $$dir ] && [ -n "$$(ls -A $$dir)" ]; then continue; fi; \
  ratarmount $$tar $$dir || { status=1; break; }; \
  mounted="$$dir $$mounted"; \
done; \
[ -n "$$status" ] || { $(1); status=$$?; }; \
for dir in $$mounted; do ratarmount -u $$dir; done; \
$(if $(strip $(2)),$(2);) \
exit $$status
endef

# Texture mesh $< using OpenMVS scene $(1), via a temporary GLB (and textures) named after the target, so
# the coarse and hi-res builds don't trip over each other
define texture
$(call with_images,\
TextureMesh -i $(1) -m $< -o $(basename $@)_temp.glb --export-type glb && \
gltf-pipeline -i $(basename $@)_temp.glb -o $@,\
rm -f $(basename $@)_temp*)
endef

# --- Meshes -------------------------------------------------------------------

coarse_full.glb: coarse_mesh.ply scene.mvs $(IMAGES)
	$(call texture,scene.mvs)

coarse_mesh.ply: scene.mvs
	ReconstructMesh -i $< -o $@

hires_full.glb: hires_mesh.ply scene_dense.mvs $(IMAGES)
	$(call texture,scene_dense.mvs)

hires_mesh.ply: scene_dense.mvs
	ReconstructMesh -i $< -o $@

scene_dense.mvs: scene.mvs $(IMAGES)
	$(call with_images,DensifyPointCloud -i $< -o $@)

scene.mvs: $(colmap_files) $(IMAGES)
	$(call with_images,InterfaceCOLMAP -i . --image-folder images -o $@)

$(colmap_files) &: atlas.json.gz $(IMAGES)
	$(call with_images,$(RASPBIAN_DIR)/colmap.py -s -d .)

# --- Atlas and viewer data ----------------------------------------------------

$(pointcloud_files) &: atlas.json.gz
	$(RASPBIAN_DIR)/make_info.py -d . -a atlas.json.gz

survey.json: survey.3d
	$(RASPBIAN_DIR)/survey_json.py $< $@

ifneq ($(wildcard atlas.osa),)
# --- a model recorded on the device ---
# merge_maps.py reuses connections.json if there is one, so drop it when it predates the atlas it came from
atlas.json.gz: atlas_raw.json.gz $(SURVEY)
	if [ connections.json -ot atlas_raw.json.gz ]; then rm -f connections.json; fi
	$(RASPBIAN_DIR)/merge_maps.py -d . -a atlas_raw.json.gz $(if $(SURVEY),-s survey.3d)

# no with_images needed: the aruco search reads images.tar directly
atlas_raw.json.gz: atlas.osa images.tar
	$(RASPBIAN_DIR)/make_json.py -d . -a $< -o $@

else ifneq ($(wildcard combine.json),)
# --- a combined model ---

# Only make in the sub-survey's own folder knows whether its atlas is out of date, so always ask it (FORCE);
# this make then sees from the atlas's time whether it changed
$(SUB_ATLASES): FORCE
	$(MAKE) -C $(@D) -f $(THIS_MAKEFILE) atlas.json.gz
FORCE:

# combine.py reuses atlas_pre_merge.json.gz/subatlas.json (the sub-surveys joined together) and
# connections.json (matches across the links) if they exist, so drop them if they predate a sub-survey's
# atlas or the settings (links and share_survey) in combine.json
atlas.json.gz: $(SUB_ATLASES) combine.json $(SURVEY)
	for f in $(SUB_ATLASES) combine.json; do \
	  [ atlas_pre_merge.json.gz -nt $$f ] || { rm -f atlas_pre_merge.json.gz subatlas.json connections.json; break; }; \
	done
	$(RASPBIAN_DIR)/combine.py -d . $(if $(SURVEY),--survey survey.3d)
endif

# only when there's a .svx: otherwise an existing survey.3d is used as it is
ifneq ($(wildcard survey.svx),)
survey.3d: survey.svx
	cavern -o $@ $<
endif
