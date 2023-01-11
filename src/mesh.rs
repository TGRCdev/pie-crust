use glam::Vec3;
use std::{
    path::Path,
    io::{ BufWriter, Write },
    fs::File,
    writeln,
    collections::HashMap,
};
use rayon::prelude::*;
use glam::vec3;
use ordered_float::NotNan;

#[derive(Debug)]
pub enum Normals {
    Vertex(Vec<Vec3>),
    Face(Vec<Vec3>),
}

#[derive(Debug)]
pub struct Mesh {
    pub indices: Option<Vec<u32>>,
    pub vertices: Vec<Vec3>,
    pub normals: Option<Normals>,
}

impl Mesh {
    pub fn write_obj_to_file(&self, filename: &impl AsRef<Path>)
    {
        let mut file = BufWriter::new(File::create(filename).unwrap());
        writeln!(file, "# Mesh generated by rusty_ground").unwrap();
        self.vertices.iter().for_each(|&vert| {
            writeln!(file, "v {} {} {}", vert.x, vert.y, vert.z).unwrap();
        });

        writeln!(file).unwrap();

        if let Some(normals) = &self.normals {
            match normals {
                Normals::Vertex(normals) => {
                    writeln!(file, "# Normals: Vertex").unwrap();
                    normals.iter().for_each(|&normal| {
                        writeln!(file, "vn {} {} {}", normal.x, normal.y, normal.z).unwrap();
                    });
                },
                Normals::Face(normals) => {
                    writeln!(file, "# Normals: Face").unwrap();
                    normals.iter().for_each(|&normal| {
                        writeln!(file, "vn {} {} {}", normal.x, normal.y, normal.z).unwrap();
                    })
                }
            }
            writeln!(file).unwrap();
        }
        else
        {
            writeln!(file, "# Normals: None\n").unwrap();
        }

        if let Some(indices) = &self.indices
        {
            if let Some(normals) = &self.normals
            {
                match normals {
                    Normals::Face(_) => {
                        indices.chunks_exact(3).enumerate().for_each(|(i, face)| {
                            writeln!(file, "f {}//{3} {}//{3} {}//{3}",
                                face[0]+1,
                                face[1]+1,
                                face[2]+1,
                                i+1,
                            ).unwrap();
                        });
                    },
                    Normals::Vertex(_) => {
                        indices.chunks_exact(3).for_each(|face| {
                            writeln!(file, "f {0}//{0} {1}//{1} {2}//{2}",
                                face[0]+1,
                                face[1]+1,
                                face[2]+1,
                            ).unwrap();
                        });
                    }
                }
            }
            else
            {
                indices.chunks_exact(3).for_each(|face| {
                    writeln!(file, "f {} {} {}",
                        face[0]+1,
                        face[1]+1,
                        face[2]+1
                    ).unwrap();
                });
            }
        }
        else
        {
            (1..self.vertices.len()+1).step_by(3)
                .map(|x| (x, x+1, x+2))
                .enumerate()
                .for_each(|(i, face)| {
                    if let Some(normals) = &self.normals
                    {
                        match normals {
                            Normals::Face(_) => {
                                writeln!(file, "f {}//{3} {}//{3} {}//{3}",
                                    face.0+1,
                                    face.1+1,
                                    face.2+1,
                                    i+1
                                ).unwrap();
                            },
                            Normals::Vertex(_) => {
                                writeln!(file, "f {0}//{0}, {1}//{1}, {2}//{2}",
                                    face.0+1,
                                    face.1+1,
                                    face.2+1,
                                ).unwrap();
                            }
                        }
                    }
                    else
                    {
                        writeln!(file, "f {} {} {}", face.0, face.1, face.2).unwrap();
                    }
                });
        }
    }

    pub fn index(&mut self) {
        if self.indices.is_some()
        {
            return;
        }

        let mut vert_map: HashMap<[NotNan<f32>; 3], (u32, u32)> = HashMap::new();
        let mut indices = Vec::new();
        self.vertices.iter().enumerate().for_each(|(i, vert)|
        {
            let cur_len = vert_map.len();
            indices.push(
                vert_map.entry(
                    vert.to_array().map(|x| NotNan::new(x).unwrap())
                ).or_insert((i as u32, cur_len as u32)).1
            );
        });

        self.indices = Some(indices);
        self.vertices.truncate(vert_map.len());

        if self.normals.as_ref().filter(|n| matches!(n, Normals::Vertex(_))).is_some()
        {
            if let Normals::Vertex(normals) = self.normals.take().unwrap()
            {
                let mut new_normals = Vec::with_capacity(vert_map.len());
                vert_map.iter().for_each(|(_,&(old_index, new_index))| {
                    new_normals[new_index as usize] = normals[old_index as usize];
                });
                new_normals.truncate(vert_map.len());

                self.normals = Some(Normals::Vertex(new_normals));
            }
        }

        vert_map.into_iter().map(|(vert, (_, new_index))| {
            (
                new_index as usize,
                Vec3::from(vert.map(|x| f32::from(x)))
            )
        })
        .for_each(|(index, vert)| {
            self.vertices[index] = vert;
        });
    }

    pub fn generate_face_normals(&mut self) {
        if self.normals.is_some()
        {
            return;
        }

        if let Some(indices) = &self.indices {
            let mut normals = Vec::with_capacity(indices.len() / 3);
            indices.par_chunks_exact(3).map(|face| {
                let edge_a = self.vertices[face[1] as usize] - self.vertices[face[0] as usize];
                let edge_b = self.vertices[face[2] as usize] - self.vertices[face[0] as usize];
                let normal = vec3(
                    (edge_a.y * edge_b.z) - (edge_a.z * edge_b.y),
                    (edge_a.z * edge_b.x) - (edge_a.x * edge_b.z),
                    (edge_a.x * edge_b.y) - (edge_a.y * edge_b.x),
                ).normalize_or_zero();

                assert!(!normal.is_nan());

                normal
            }).collect_into_vec(&mut normals);

            self.normals = Some(Normals::Face(normals));
        }
        else
        {
            let mut normals = Vec::with_capacity(self.vertices.len() / 3);
            (0..self.vertices.len()/3).into_par_iter()
                .map(|x| [x*3, (x*3) + 1, (x*3) + 2])
                .map(|face| {
                    let edge_a = self.vertices[face[1] as usize] - self.vertices[face[0] as usize];
                    let edge_b = self.vertices[face[2] as usize] - self.vertices[face[0] as usize];
                    vec3(
                        (edge_a.y * edge_b.z) - (edge_a.z * edge_b.y),
                        (edge_a.z * edge_b.x) - (edge_a.x * edge_b.z),
                        (edge_a.x * edge_b.y) - (edge_a.y * edge_b.x),
                    ).normalize_or_zero()
                }).collect_into_vec(&mut normals);
            
            self.normals = Some(Normals::Face(normals));
        }
    }

    pub fn generate_vertex_normals(&mut self) {
        if self.normals.is_none()
        {
            self.generate_face_normals();
        }

        {
            if self.indices.is_none()
            {
                self.index();
            }

            let normals;
            match self.normals.as_mut().unwrap()
            {
                Normals::Vertex(_) => { return } // Already have vertex normals
                Normals::Face(face_normals) => {
                    normals = face_normals;
                }
            }

            let mut normal_sums: HashMap<u32, (u32, Vec3)> = HashMap::new();

            if let Some(indices) = &self.indices {
                indices.chunks_exact(3).enumerate().for_each(|(i, face)| {
                    for &vert in face {
                        let sum_data = normal_sums.entry(vert).or_default();
                        sum_data.0 += 1;
                        sum_data.1 += normals[i];
                    }
                });
            }

            normals.resize(self.vertices.len(), Vec3::ZERO);
            normal_sums.into_iter().for_each(|(i, (count, normal_sum))| {
                if count > 0
                {
                    normals[i as usize] = normal_sum / (count as f32);
                }
                else
                {
                    normals[i as usize] = Vec3::ZERO;
                }
            });
        } // Drop normals mut ref

        self.normals = match self.normals.take().unwrap() {
            Normals::Vertex(normals) => Some(Normals::Vertex(normals)),
            Normals::Face(normals) => Some(Normals::Vertex(normals)),
        }
    }
}