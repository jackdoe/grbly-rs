use std::fs;
use std::io::Write;
use std::path::PathBuf;

#[derive(Clone, Debug, PartialEq)]
pub struct HeightMap {
    pub bbox_min: (f32, f32),
    pub bbox_max: (f32, f32),
    pub grid_x: u32,
    pub grid_y: u32,
    pub z: Vec<f32>,
}

pub fn grid_point(
    bbox_min: (f32, f32),
    bbox_max: (f32, f32),
    grid_x: u32,
    grid_y: u32,
    idx: usize,
) -> (f32, f32) {
    let i = (idx % grid_x as usize) as u32;
    let j = (idx / grid_x as usize) as u32;
    let fx = i as f32 / (grid_x - 1).max(1) as f32;
    let fy = j as f32 / (grid_y - 1).max(1) as f32;
    (
        bbox_min.0 + fx * (bbox_max.0 - bbox_min.0),
        bbox_min.1 + fy * (bbox_max.1 - bbox_min.1),
    )
}

impl HeightMap {
    pub fn new(
        bbox_min: (f32, f32),
        bbox_max: (f32, f32),
        grid_x: u32,
        grid_y: u32,
        z: Vec<f32>,
    ) -> Result<Self, String> {
        if grid_x < 2 || grid_y < 2 {
            return Err("grid_x and grid_y must be >= 2".into());
        }
        if bbox_max.0 <= bbox_min.0 || bbox_max.1 <= bbox_min.1 {
            return Err("bbox_max must be strictly greater than bbox_min".into());
        }
        if z.len() != (grid_x * grid_y) as usize {
            return Err(format!(
                "z length {} does not match grid {}x{}",
                z.len(),
                grid_x,
                grid_y
            ));
        }
        Ok(Self {
            bbox_min,
            bbox_max,
            grid_x,
            grid_y,
            z,
        })
    }

    pub fn dz(&self, x: f32, y: f32) -> f32 {
        let x = x.clamp(self.bbox_min.0, self.bbox_max.0);
        let y = y.clamp(self.bbox_min.1, self.bbox_max.1);
        let span_x = self.bbox_max.0 - self.bbox_min.0;
        let span_y = self.bbox_max.1 - self.bbox_min.1;
        let fx = (x - self.bbox_min.0) / span_x * (self.grid_x - 1) as f32;
        let fy = (y - self.bbox_min.1) / span_y * (self.grid_y - 1) as f32;
        let i = (fx as u32).min(self.grid_x - 2);
        let j = (fy as u32).min(self.grid_y - 2);
        let u = (fx - i as f32).clamp(0.0, 1.0);
        let v = (fy - j as f32).clamp(0.0, 1.0);
        let gx = self.grid_x as usize;
        let z00 = self.z[j as usize * gx + i as usize];
        let z10 = self.z[j as usize * gx + (i + 1) as usize];
        let z01 = self.z[(j + 1) as usize * gx + i as usize];
        let z11 = self.z[(j + 1) as usize * gx + (i + 1) as usize];
        (1.0 - u) * (1.0 - v) * z00
            + u * (1.0 - v) * z10
            + (1.0 - u) * v * z01
            + u * v * z11
    }

    pub fn z_min_max(&self) -> (f32, f32) {
        let mut lo = f32::INFINITY;
        let mut hi = f32::NEG_INFINITY;
        for &z in &self.z {
            lo = lo.min(z);
            hi = hi.max(z);
        }
        (lo, hi)
    }

    pub fn serialize(&self) -> String {
        let mut s = String::new();
        s.push_str("HEIGHTMAP v1\n");
        s.push_str(&format!(
            "bbox {:.4} {:.4} {:.4} {:.4}\n",
            self.bbox_min.0, self.bbox_min.1, self.bbox_max.0, self.bbox_max.1
        ));
        s.push_str(&format!("grid {} {}\n", self.grid_x, self.grid_y));
        for j in 0..self.grid_y {
            let mut row = String::new();
            for i in 0..self.grid_x {
                if i > 0 {
                    row.push(' ');
                }
                row.push_str(&format!("{:.4}", self.z[(j * self.grid_x + i) as usize]));
            }
            row.push('\n');
            s.push_str(&row);
        }
        s
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header = lines.next().ok_or_else(|| "empty input".to_string())?;
        if header.trim() != "HEIGHTMAP v1" {
            return Err(format!("bad header {header}"));
        }
        let bbox_line = lines.next().ok_or_else(|| "missing bbox".to_string())?;
        let bbox_parts: Vec<&str> = bbox_line.split_whitespace().collect();
        if bbox_parts.len() != 5 || bbox_parts[0] != "bbox" {
            return Err(format!("bad bbox {bbox_line}"));
        }
        let bbox_min = (
            bbox_parts[1].parse::<f32>().map_err(|e| e.to_string())?,
            bbox_parts[2].parse::<f32>().map_err(|e| e.to_string())?,
        );
        let bbox_max = (
            bbox_parts[3].parse::<f32>().map_err(|e| e.to_string())?,
            bbox_parts[4].parse::<f32>().map_err(|e| e.to_string())?,
        );
        let grid_line = lines.next().ok_or_else(|| "missing grid".to_string())?;
        let grid_parts: Vec<&str> = grid_line.split_whitespace().collect();
        if grid_parts.len() != 3 || grid_parts[0] != "grid" {
            return Err(format!("bad grid {grid_line}"));
        }
        let grid_x: u32 = grid_parts[1]
            .parse()
            .map_err(|_| "bad grid_x".to_string())?;
        let grid_y: u32 = grid_parts[2]
            .parse()
            .map_err(|_| "bad grid_y".to_string())?;
        let mut z = Vec::with_capacity((grid_x * grid_y) as usize);
        for _ in 0..grid_y {
            let row = lines.next().ok_or_else(|| "missing row".to_string())?;
            for tok in row.split_whitespace() {
                z.push(tok.parse::<f32>().map_err(|_| format!("bad z {tok}"))?);
            }
        }
        Self::new(bbox_min, bbox_max, grid_x, grid_y, z)
    }
}

pub fn cache_path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    let mut p = PathBuf::from(home);
    p.push(".cache");
    p.push("grbly");
    p.push("heightmap.txt");
    Some(p)
}

pub fn save_cached(map: &HeightMap) -> Result<(), String> {
    let path = cache_path().ok_or_else(|| "no HOME".to_string())?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let mut f = fs::File::create(&path).map_err(|e| e.to_string())?;
    f.write_all(map.serialize().as_bytes())
        .map_err(|e| e.to_string())?;
    Ok(())
}

pub fn load_cached() -> Option<HeightMap> {
    let path = cache_path()?;
    let text = fs::read_to_string(&path).ok()?;
    HeightMap::parse(&text).ok()
}

pub fn clear_cached() {
    if let Some(path) = cache_path() {
        let _ = fs::remove_file(path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_map() -> HeightMap {
        HeightMap::new((0.0, 0.0), (10.0, 10.0), 2, 2, vec![0.0, 0.0, 0.0, 0.0]).unwrap()
    }

    #[test]
    fn flat_map_dz_zero_everywhere() {
        let m = flat_map();
        assert_eq!(m.dz(0.0, 0.0), 0.0);
        assert_eq!(m.dz(5.0, 5.0), 0.0);
        assert_eq!(m.dz(10.0, 10.0), 0.0);
    }

    #[test]
    fn corners_returned_exactly() {
        let m = HeightMap::new(
            (0.0, 0.0),
            (10.0, 10.0),
            2,
            2,
            vec![1.0, 2.0, 3.0, 4.0],
        )
        .unwrap();
        assert_eq!(m.dz(0.0, 0.0), 1.0);
        assert_eq!(m.dz(10.0, 0.0), 2.0);
        assert_eq!(m.dz(0.0, 10.0), 3.0);
        assert_eq!(m.dz(10.0, 10.0), 4.0);
    }

    #[test]
    fn bilinear_midpoint() {
        let m = HeightMap::new(
            (0.0, 0.0),
            (10.0, 10.0),
            2,
            2,
            vec![0.0, 1.0, 1.0, 2.0],
        )
        .unwrap();
        assert!((m.dz(5.0, 5.0) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn dz_clamps_outside_bbox() {
        let m = HeightMap::new(
            (0.0, 0.0),
            (10.0, 10.0),
            2,
            2,
            vec![1.0, 2.0, 3.0, 4.0],
        )
        .unwrap();
        assert_eq!(m.dz(-100.0, -100.0), 1.0);
        assert_eq!(m.dz(100.0, 100.0), 4.0);
    }

    #[test]
    fn serialize_roundtrip() {
        let m = HeightMap::new(
            (1.0, 2.0),
            (11.0, 22.0),
            3,
            3,
            vec![0.0, 0.1, 0.2, 0.05, 0.15, 0.25, 0.1, 0.2, 0.3],
        )
        .unwrap();
        let s = m.serialize();
        let m2 = HeightMap::parse(&s).unwrap();
        assert_eq!(m, m2);
    }

    #[test]
    fn rejects_invalid_grid() {
        assert!(HeightMap::new((0.0, 0.0), (1.0, 1.0), 1, 2, vec![0.0, 0.0]).is_err());
    }

    #[test]
    fn rejects_length_mismatch() {
        assert!(HeightMap::new((0.0, 0.0), (1.0, 1.0), 2, 2, vec![0.0, 0.0]).is_err());
    }
}
