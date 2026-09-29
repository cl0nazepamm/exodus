// Exodus - FBX/OBJ input through ufbx.
#include "exodus.h"

#include <filesystem>
#include <map>
#include <set>

#include "ufbx.h"

namespace fs = std::filesystem;

static std::string str(ufbx_string s) { return std::string(s.data, s.length); }

static bool is_file(const fs::path& p) {
    std::error_code ec;
    return !p.empty() && fs::is_regular_file(p, ec);
}

// Resolve a texture to a file on disk. Order: ufbx-resolved name, absolute name, relative to the
// FBX folder, the "<stem>.fbm" media folder next to the FBX, then the bare file name next to the FBX.
static std::string resolve_texture(const ufbx_texture* tex, const fs::path& fbxPath) {
    const fs::path dir = fbxPath.parent_path();
    std::vector<fs::path> tries;
    auto add = [&](ufbx_string s) { if (s.length) tries.push_back(upath(str(s))); };
    add(tex->filename);
    add(tex->absolute_filename);
    if (tex->relative_filename.length) tries.push_back(dir / upath(str(tex->relative_filename)));
    fs::path base;
    for (ufbx_string s : {tex->filename, tex->absolute_filename, tex->relative_filename}) {
        if (!s.length) continue;
        std::string v = str(s);
        size_t cut = v.find_last_of("/\\");
        base = upath(cut == std::string::npos ? v : v.substr(cut + 1));
        break;
    }
    if (!base.empty()) {
        fs::path fbm = fbxPath;
        fbm.replace_extension(".fbm");
        tries.push_back(fbm / base);
        tries.push_back(dir / base);
    }
    for (const fs::path& p : tries)
        if (is_file(p)) return ustr(fs::absolute(p));
    return {};
}

static std::string texture_label(const ufbx_texture* tex) {
    for (ufbx_string s : {tex->relative_filename, tex->filename, tex->absolute_filename}) {
        if (!s.length) continue;
        std::string v = str(s);
        size_t cut = v.find_last_of("/\\");
        return cut == std::string::npos ? v : v.substr(cut + 1);
    }
    return str(tex->name);
}

bool load_fbx(const std::string& path, const std::string& uvSet, InScene& out, std::string& err) {
    ufbx_load_opts opts = {};
    opts.ignore_animation = true;
    opts.load_external_files = false;
    opts.generate_missing_normals = true;

    ufbx_error uerr;
    ufbx_scene* scene = ufbx_load_file(path.c_str(), &opts, &uerr);
    if (!scene) {
        char buf[1024];
        ufbx_format_error(buf, sizeof(buf), &uerr);
        err = buf;
        return false;
    }

    const fs::path fbxPath = fs::absolute(upath(path));
    out.sourcePath = ustr(fbxPath);

    std::map<const ufbx_material*, uint32_t> matIndex;
    auto material_id = [&](const ufbx_material* m) -> uint32_t {
        auto it = matIndex.find(m);
        if (it != matIndex.end()) return it->second;
        uint32_t id = (uint32_t)out.materials.size();
        matIndex[m] = id;
        InMaterial im;
        if (!m) {
            im.name = "(no material)";
        } else {
            im.name = str(m->name);
            for (size_t i = 0; i < m->textures.count; ++i) {
                const ufbx_material_texture& mt = m->textures.data[i];
                const ufbx_texture* tex = mt.texture;
                if (tex && tex->type != UFBX_TEXTURE_FILE && tex->file_textures.count)
                    tex = tex->file_textures.data[0];
                if (!tex || tex->type != UFBX_TEXTURE_FILE) continue;
                InTexture t;
                t.channel = str(mt.material_prop);
                t.label = texture_label(tex);
                t.path = resolve_texture(tex, fbxPath);
                if (t.path.empty() && tex->content.size) {
                    const uint8_t* p = (const uint8_t*)tex->content.data;
                    t.embedded.assign(p, p + tex->content.size);
                }
                im.textures.push_back(std::move(t));
            }
        }
        out.materials.push_back(std::move(im));
        return id;
    };

    std::set<const ufbx_mesh*> seen;
    for (size_t ni = 0; ni < scene->nodes.count; ++ni) {
        const ufbx_node* node = scene->nodes.data[ni];
        const ufbx_mesh* mesh = node->mesh;
        if (!mesh || !seen.insert(mesh).second) continue;

        const ufbx_vertex_vec2* uv = &mesh->vertex_uv;
        if (!uvSet.empty()) {
            uv = nullptr;
            for (size_t i = 0; i < mesh->uv_sets.count; ++i)
                if (str(mesh->uv_sets.data[i].name) == uvSet) uv = &mesh->uv_sets.data[i].vertex_uv;
        }
        if (!uv || !uv->exists) continue;  // nothing to pack on a mesh without UVs

        InMesh im;
        im.name = str(node->name);
        const ufbx_matrix& xf = node->geometry_to_world;
        const ufbx_matrix nxf = ufbx_matrix_for_normals(&xf);

        im.positions.reserve(mesh->vertex_position.values.count);
        for (size_t i = 0; i < mesh->vertex_position.values.count; ++i) {
            ufbx_vec3 p = ufbx_transform_position(&xf, mesh->vertex_position.values.data[i]);
            im.positions.push_back({p.x, p.y, p.z});
        }
        if (mesh->vertex_normal.exists) {
            for (size_t i = 0; i < mesh->vertex_normal.values.count; ++i) {
                ufbx_vec3 n = ufbx_transform_direction(&nxf, mesh->vertex_normal.values.data[i]);
                im.normals.push_back({n.x, n.y, n.z});
            }
        }
        for (size_t i = 0; i < uv->values.count; ++i)
            im.uvs.push_back({uv->values.data[i].x, uv->values.data[i].y});

        // Instance materials win over mesh materials when the node carries a full set.
        const ufbx_material_list& mats =
            node->materials.count >= mesh->materials.count ? node->materials : mesh->materials;

        im.cornerPos.resize(mesh->num_indices);
        im.cornerUv.resize(mesh->num_indices);
        if (mesh->vertex_normal.exists) im.cornerNrm.resize(mesh->num_indices);
        for (size_t i = 0; i < mesh->num_indices; ++i) {
            im.cornerPos[i] = mesh->vertex_position.indices.data[i];
            im.cornerUv[i] = uv->indices.data[i];
            if (mesh->vertex_normal.exists) im.cornerNrm[i] = mesh->vertex_normal.indices.data[i];
        }

        std::vector<uint32_t> triIdx(mesh->max_face_triangles * 3);
        for (size_t fi = 0; fi < mesh->faces.count; ++fi) {
            const ufbx_face face = mesh->faces.data[fi];
            if (face.num_indices < 3) continue;
            InFace f;
            f.first = face.index_begin;
            f.count = face.num_indices;
            uint32_t slot = mesh->face_material.count ? mesh->face_material.data[fi] : 0;
            f.material = material_id(slot < mats.count ? mats.data[slot] : nullptr);
            uint32_t faceId = (uint32_t)im.faces.size();
            im.faces.push_back(f);
            uint32_t nt = ufbx_triangulate_face(triIdx.data(), triIdx.size(), mesh, face);
            for (uint32_t t = 0; t < nt; ++t) {
                im.tris.push_back({triIdx[t * 3 + 0], triIdx[t * 3 + 1], triIdx[t * 3 + 2]});
                im.triFace.push_back(faceId);
            }
        }
        out.meshes.push_back(std::move(im));
    }

    // Constant values for every channel name any material maps, so unmapped materials can fill it.
    std::set<std::string> channels;
    for (const InMaterial& m : out.materials)
        for (const InTexture& t : m.textures) channels.insert(t.channel);
    for (const auto& [mat, id] : matIndex) {
        if (!mat) continue;
        for (const std::string& ch : channels) {
            const ufbx_prop* p = ufbx_find_prop(&mat->props, ch.c_str());
            if (!p) continue;
            Vec4d v;
            if (p->type == UFBX_PROP_NUMBER || p->type == UFBX_PROP_INTEGER || p->type == UFBX_PROP_BOOLEAN)
                v = {p->value_real, p->value_real, p->value_real, 1.0};
            else
                v = {p->value_vec4.x, p->value_vec4.y, p->value_vec4.z, 1.0};
            out.materials[id].constants.push_back({ch, v});
        }
    }

    ufbx_free_scene(scene);
    return true;
}
