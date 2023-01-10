use glam::Vec3;
use lerp::Lerp;

pub const EDGE_TABLE: [usize; 256] = [
	0x0  , 0x103, 0x809, 0x90a, 0x130, 0x33 , 0x939, 0x83a, 
	0x890, 0x993, 0x99 , 0x19a, 0x9a0, 0x8a3, 0x1a9, 0xaa , 
	0x206, 0x305, 0xa0f, 0xb0c, 0x336, 0x235, 0xb3f, 0xa3c, 
	0xa96, 0xb95, 0x29f, 0x39c, 0xba6, 0xaa5, 0x3af, 0x2ac, 
	0x40c, 0x50f, 0xc05, 0xd06, 0x53c, 0x43f, 0xd35, 0xc36, 
	0xc9c, 0xd9f, 0x495, 0x596, 0xdac, 0xcaf, 0x5a5, 0x4a6, 
	0x60a, 0x709, 0xe03, 0xf00, 0x73a, 0x639, 0xf33, 0xe30, 
	0xe9a, 0xf99, 0x693, 0x790, 0xfaa, 0xea9, 0x7a3, 0x6a0, 
	0x260, 0x363, 0xa69, 0xb6a, 0x350, 0x253, 0xb59, 0xa5a, 
	0xaf0, 0xbf3, 0x2f9, 0x3fa, 0xbc0, 0xac3, 0x3c9, 0x2ca, 
	0x66 , 0x165, 0x86f, 0x96c, 0x156, 0x55 , 0x95f, 0x85c, 
	0x8f6, 0x9f5, 0xff , 0x1fc, 0x9c6, 0x8c5, 0x1cf, 0xcc , 
	0x66c, 0x76f, 0xe65, 0xf66, 0x75c, 0x65f, 0xf55, 0xe56, 
	0xefc, 0xfff, 0x6f5, 0x7f6, 0xfcc, 0xecf, 0x7c5, 0x6c6, 
	0x46a, 0x569, 0xc63, 0xd60, 0x55a, 0x459, 0xd53, 0xc50, 
	0xcfa, 0xdf9, 0x4f3, 0x5f0, 0xdca, 0xcc9, 0x5c3, 0x4c0, 
	0x4c0, 0x5c3, 0xcc9, 0xdca, 0x5f0, 0x4f3, 0xdf9, 0xcfa, 
	0xc50, 0xd53, 0x459, 0x55a, 0xd60, 0xc63, 0x569, 0x46a, 
	0x6c6, 0x7c5, 0xecf, 0xfcc, 0x7f6, 0x6f5, 0xfff, 0xefc, 
	0xe56, 0xf55, 0x65f, 0x75c, 0xf66, 0xe65, 0x76f, 0x66c, 
	0xcc , 0x1cf, 0x8c5, 0x9c6, 0x1fc, 0xff , 0x9f5, 0x8f6, 
	0x85c, 0x95f, 0x55 , 0x156, 0x96c, 0x86f, 0x165, 0x66 , 
	0x2ca, 0x3c9, 0xac3, 0xbc0, 0x3fa, 0x2f9, 0xbf3, 0xaf0, 
	0xa5a, 0xb59, 0x253, 0x350, 0xb6a, 0xa69, 0x363, 0x260, 
	0x6a0, 0x7a3, 0xea9, 0xfaa, 0x790, 0x693, 0xf99, 0xe9a, 
	0xe30, 0xf33, 0x639, 0x73a, 0xf00, 0xe03, 0x709, 0x60a, 
	0x4a6, 0x5a5, 0xcaf, 0xdac, 0x596, 0x495, 0xd9f, 0xc9c, 
	0xc36, 0xd35, 0x43f, 0x53c, 0xd06, 0xc05, 0x50f, 0x40c, 
	0x2ac, 0x3af, 0xaa5, 0xba6, 0x39c, 0x29f, 0xb95, 0xa96, 
	0xa3c, 0xb3f, 0x235, 0x336, 0xb0c, 0xa0f, 0x305, 0x206, 
	0xaa , 0x1a9, 0x8a3, 0x9a0, 0x19a, 0x99 , 0x993, 0x890, 
	0x83a, 0x939, 0x33 , 0x130, 0x90a, 0x809, 0x103, 0x0  , 
];

pub const TRI_TABLE: [[i8; 15]; 256] = [
	[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  8,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1, 11,  3,  8, 11,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5,  4,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5,  0,  1,  4,  0,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  3,  0,  4,  8,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  5,  4, 11,  3,  5,  3,  1,  5, -1, -1, -1, -1, -1, -1],
	[11,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 0,  1,  8, 11,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  4,  7,  0,  4,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 4,  1,  8,  4,  7,  1,  7,  3,  1, -1, -1, -1, -1, -1, -1],
	[ 8,  7, 11,  5,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 0,  7, 11,  0,  1,  7,  1,  5,  7, -1, -1, -1, -1, -1, -1],
	[ 8,  3,  0,  8,  5,  3,  5,  7,  3, -1, -1, -1, -1, -1, -1],
	[ 3,  1,  7,  1,  5,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  9,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  8,  0,  9,  8,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  9,  1,  3,  0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2, 11,  3,  2,  9, 11,  9,  8, 11, -1, -1, -1, -1, -1, -1],
	[ 1,  2,  9,  8,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5,  2,  9,  5,  4,  2,  4,  0,  2, -1, -1, -1, -1, -1, -1],
	[ 3,  0, 11,  1,  2,  9,  4,  8,  5, -1, -1, -1, -1, -1, -1],
	[11,  5,  4,  3,  5, 11,  3,  9,  5,  3,  2,  9, -1, -1, -1],
	[ 2,  9,  1,  7, 11,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 8,  2,  9,  8,  0,  2, 11,  4,  7, -1, -1, -1, -1, -1, -1],
	[ 3,  4,  7,  3,  0,  4,  1,  2,  9, -1, -1, -1, -1, -1, -1],
	[ 2,  9,  8,  2,  8,  7,  2,  7,  3,  7,  8,  4, -1, -1, -1],
	[ 8,  7, 11,  8,  5,  7,  9,  1,  2, -1, -1, -1, -1, -1, -1],
	[ 9,  0,  2,  9,  7,  0,  9,  5,  7, 11,  0,  7, -1, -1, -1],
	[ 2,  9,  1,  3,  0,  5,  3,  5,  7,  5,  0,  8, -1, -1, -1],
	[ 2,  9,  5,  2,  5,  3,  3,  5,  7, -1, -1, -1, -1, -1, -1],
	[ 2,  3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3, 10,  2,  0,  1,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  0, 11,  2,  0, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1, 10,  2,  1,  8, 10,  8, 11, 10, -1, -1, -1, -1, -1, -1],
	[ 2,  3, 10,  8,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 0,  5,  4,  0,  1,  5,  2,  3, 10, -1, -1, -1, -1, -1, -1],
	[ 0, 10,  2,  0, 11, 10,  4,  8,  5, -1, -1, -1, -1, -1, -1],
	[ 2,  1,  5,  2,  5, 11,  2, 11, 10,  4, 11,  5, -1, -1, -1],
	[10,  2,  3,  7, 11,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  3, 10,  0,  1,  8,  7, 11,  4, -1, -1, -1, -1, -1, -1],
	[10,  4,  7, 10,  2,  4,  2,  0,  4, -1, -1, -1, -1, -1, -1],
	[ 1,  8,  4,  2,  1,  4,  2,  4,  7,  2,  7, 10, -1, -1, -1],
	[ 7,  8,  5,  7, 11,  8,  3, 10,  2, -1, -1, -1, -1, -1, -1],
	[10,  2,  3,  7, 11,  1,  7,  1,  5,  1, 11,  0, -1, -1, -1],
	[10,  5,  7, 10,  0,  5, 10,  2,  0,  0,  8,  5, -1, -1, -1],
	[10,  2,  1, 10,  1,  7,  7,  1,  5, -1, -1, -1, -1, -1, -1],
	[ 3,  9,  1, 10,  9,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  8,  0,  3, 10,  8, 10,  9,  8, -1, -1, -1, -1, -1, -1],
	[ 0,  9,  1,  0, 11,  9, 11, 10,  9, -1, -1, -1, -1, -1, -1],
	[10,  9, 11, 11,  9,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 9,  3, 10,  9,  1,  3,  8,  5,  4, -1, -1, -1, -1, -1, -1],
	[ 3,  4,  0,  3,  9,  4,  3, 10,  9,  9,  5,  4, -1, -1, -1],
	[ 4,  8,  5,  0, 11,  1, 11,  9,  1, 11, 10,  9, -1, -1, -1],
	[ 5,  4, 11,  5, 11,  9,  9, 11, 10, -1, -1, -1, -1, -1, -1],
	[ 3,  9,  1,  3, 10,  9,  7, 11,  4, -1, -1, -1, -1, -1, -1],
	[ 7, 11,  4,  3, 10,  0, 10,  8,  0, 10,  9,  8, -1, -1, -1],
	[ 7, 10,  9,  7,  9,  0,  7,  0,  4,  1,  0,  9, -1, -1, -1],
	[ 4,  7, 10,  4, 10,  8,  8, 10,  9, -1, -1, -1, -1, -1, -1],
	[11,  5,  7, 11,  8,  5, 10,  9,  3,  9,  1,  3, -1, -1, -1],
	[10,  9,  0, 10,  0,  3,  9,  5,  0, 11,  0,  7,  5,  7,  0],
	[ 5,  7,  0,  5,  0,  8,  7, 10,  0,  1,  0,  9, 10,  9,  0],
	[10,  9,  5,  7, 10,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 6,  5,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 9,  6,  5,  1,  8,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  0, 11,  9,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1, 11,  3,  1,  8, 11,  5,  9,  6, -1, -1, -1, -1, -1, -1],
	[ 4,  9,  6,  8,  9,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 9,  0,  1,  9,  6,  0,  6,  4,  0, -1, -1, -1, -1, -1, -1],
	[ 4,  9,  6,  4,  8,  9,  0, 11,  3, -1, -1, -1, -1, -1, -1],
	[ 9,  3,  1,  9,  4,  3,  9,  6,  4,  4, 11,  3, -1, -1, -1],
	[ 7, 11,  4,  6,  5,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  8,  0,  5,  9,  6, 11,  4,  7, -1, -1, -1, -1, -1, -1],
	[ 4,  3,  0,  4,  7,  3,  6,  5,  9, -1, -1, -1, -1, -1, -1],
	[ 9,  6,  5,  1,  8,  7,  1,  7,  3,  7,  8,  4, -1, -1, -1],
	[ 7,  9,  6,  7, 11,  9, 11,  8,  9, -1, -1, -1, -1, -1, -1],
	[ 0,  7, 11,  1,  7,  0,  1,  6,  7,  1,  9,  6, -1, -1, -1],
	[ 0,  7,  3,  0,  9,  7,  0,  8,  9,  6,  7,  9, -1, -1, -1],
	[ 9,  6,  7,  9,  7,  1,  1,  7,  3, -1, -1, -1, -1, -1, -1],
	[ 6,  1,  2,  5,  1,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 8,  6,  5,  8,  0,  6,  0,  2,  6, -1, -1, -1, -1, -1, -1],
	[ 1,  6,  5,  1,  2,  6,  3,  0, 11, -1, -1, -1, -1, -1, -1],
	[ 3,  8, 11,  3,  6,  8,  3,  2,  6,  5,  8,  6, -1, -1, -1],
	[ 1,  4,  8,  1,  2,  4,  2,  6,  4, -1, -1, -1, -1, -1, -1],
	[ 2,  6,  0,  6,  4,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  0, 11,  1,  2,  8,  2,  4,  8,  2,  6,  4, -1, -1, -1],
	[11,  3,  2, 11,  2,  4,  4,  2,  6, -1, -1, -1, -1, -1, -1],
	[ 6,  1,  2,  6,  5,  1,  4,  7, 11, -1, -1, -1, -1, -1, -1],
	[ 7, 11,  4,  6,  5,  0,  6,  0,  2,  0,  5,  8, -1, -1, -1],
	[ 3,  0,  7,  7,  0,  4,  1,  2,  6,  1,  6,  5, -1, -1, -1],
	[ 7,  3,  8,  7,  8,  4,  3,  2,  8,  5,  8,  6,  2,  6,  8],
	[ 7,  2,  6,  7,  8,  2,  7, 11,  8,  8,  1,  2, -1, -1, -1],
	[ 7, 11,  0,  7,  0,  6,  6,  0,  2, -1, -1, -1, -1, -1, -1],
	[ 2,  6,  8,  2,  8,  1,  6,  7,  8,  0,  8,  3,  7,  3,  8],
	[ 2,  6,  7,  3,  2,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  3, 10,  9,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  2,  3,  9,  6,  5,  0,  1,  8, -1, -1, -1, -1, -1, -1],
	[10,  0, 11, 10,  2,  0,  9,  6,  5, -1, -1, -1, -1, -1, -1],
	[ 6,  5,  9, 10,  2,  8, 10,  8, 11,  8,  2,  1, -1, -1, -1],
	[ 9,  4,  8,  9,  6,  4, 10,  2,  3, -1, -1, -1, -1, -1, -1],
	[ 3, 10,  2,  0,  1,  6,  0,  6,  4,  6,  1,  9, -1, -1, -1],
	[ 6,  8,  9,  6,  4,  8,  2,  0, 10,  0, 11, 10, -1, -1, -1],
	[ 6,  4,  1,  6,  1,  9,  4, 11,  1,  2,  1, 10, 11, 10,  1],
	[ 3, 10,  2,  7, 11,  4,  9,  6,  5, -1, -1, -1, -1, -1, -1],
	[ 2,  3, 10,  6,  5,  9,  0,  1,  8,  7, 11,  4, -1, -1, -1],
	[ 9,  6,  5, 10,  2,  7,  2,  4,  7,  2,  0,  4, -1, -1, -1],
	[ 2,  7, 10,  2,  4,  7,  2,  1,  4,  8,  4,  1,  9,  6,  5],
	[ 2,  3, 10,  9,  6, 11,  9, 11,  8, 11,  6,  7, -1, -1, -1],
	[ 1, 11,  0,  1,  7, 11,  1,  9,  7,  6,  7,  9,  2,  3, 10],
	[ 2,  0,  7,  2,  7, 10,  0,  8,  7,  6,  7,  9,  8,  9,  7],
	[ 9,  6,  7,  9,  7,  1, 10,  2,  7,  2,  1,  7, -1, -1, -1],
	[ 6,  3, 10,  6,  5,  3,  5,  1,  3, -1, -1, -1, -1, -1, -1],
	[ 6,  5,  8, 10,  6,  8, 10,  8,  0, 10,  0,  3, -1, -1, -1],
	[ 0, 11, 10,  0, 10,  5,  0,  5,  1,  5, 10,  6, -1, -1, -1],
	[ 6,  5,  8,  6,  8, 10, 10,  8, 11, -1, -1, -1, -1, -1, -1],
	[10,  6,  4, 10,  4,  1, 10,  1,  3,  8,  1,  4, -1, -1, -1],
	[ 3, 10,  6,  3,  6,  0,  0,  6,  4, -1, -1, -1, -1, -1, -1],
	[11, 10,  1, 11,  1,  0, 10,  6,  1,  8,  1,  4,  6,  4,  1],
	[11, 10,  6,  4, 11,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  4,  7,  3, 10,  5,  3,  5,  1,  5, 10,  6, -1, -1, -1],
	[10,  0,  3, 10,  8,  0, 10,  6,  8,  5,  8,  6,  7, 11,  4],
	[ 5,  1, 10,  5, 10,  6,  1,  0, 10,  7, 10,  4,  0,  4, 10],
	[ 4,  7, 10,  4, 10,  8,  6,  5, 10,  5,  8, 10, -1, -1, -1],
	[11,  8,  6, 11,  6,  7,  8,  1,  6, 10,  6,  3,  1,  3,  6],
	[ 7, 11,  0,  7,  0,  6,  3, 10,  0, 10,  6,  0, -1, -1, -1],
	[10,  6,  7,  0,  8,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  6,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  7,  6,  0,  1,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  0, 11, 10,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  1,  8, 11,  3,  1, 10,  7,  6, -1, -1, -1, -1, -1, -1],
	[ 6, 10,  7,  5,  4,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5,  0,  1,  5,  4,  0,  7,  6, 10, -1, -1, -1, -1, -1, -1],
	[10,  7,  6, 11,  3,  0,  5,  4,  8, -1, -1, -1, -1, -1, -1],
	[10,  7,  6, 11,  3,  4,  3,  5,  4,  3,  1,  5, -1, -1, -1],
	[11,  6, 10,  4,  6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  6, 10, 11,  4,  6,  8,  0,  1, -1, -1, -1, -1, -1, -1],
	[ 3,  6, 10,  3,  0,  6,  0,  4,  6, -1, -1, -1, -1, -1, -1],
	[10,  4,  6, 10,  1,  4, 10,  3,  1,  8,  4,  1, -1, -1, -1],
	[ 6,  8,  5,  6, 10,  8, 10, 11,  8, -1, -1, -1, -1, -1, -1],
	[ 0, 10, 11,  0,  5, 10,  0,  1,  5,  5,  6, 10, -1, -1, -1],
	[ 6,  8,  5, 10,  8,  6, 10,  0,  8, 10,  3,  0, -1, -1, -1],
	[ 6, 10,  3,  6,  3,  5,  5,  3,  1, -1, -1, -1, -1, -1, -1],
	[10,  7,  6,  2,  9,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  8,  0,  2,  9,  8,  6, 10,  7, -1, -1, -1, -1, -1, -1],
	[ 1,  2,  9,  3,  0, 11,  6, 10,  7, -1, -1, -1, -1, -1, -1],
	[ 6, 10,  7,  2,  9,  3,  9, 11,  3,  9,  8, 11, -1, -1, -1],
	[ 2,  9,  1,  6, 10,  7,  8,  5,  4, -1, -1, -1, -1, -1, -1],
	[10,  7,  6,  2,  9,  4,  2,  4,  0,  4,  9,  5, -1, -1, -1],
	[ 4,  8,  5,  3,  0, 11,  2,  9,  1,  6, 10,  7, -1, -1, -1],
	[ 3,  4, 11,  3,  5,  4,  3,  2,  5,  9,  5,  2, 10,  7,  6],
	[ 6, 11,  4,  6, 10, 11,  2,  9,  1, -1, -1, -1, -1, -1, -1],
	[ 2,  9,  0,  0,  9,  8,  6, 10, 11,  6, 11,  4, -1, -1, -1],
	[ 1,  2,  9,  3,  0, 10,  0,  6, 10,  0,  4,  6, -1, -1, -1],
	[ 9,  8,  3,  9,  3,  2,  8,  4,  3, 10,  3,  6,  4,  6,  3],
	[ 2,  9,  1,  6, 10,  5, 10,  8,  5, 10, 11,  8, -1, -1, -1],
	[10, 11,  5, 10,  5,  6, 11,  0,  5,  9,  5,  2,  0,  2,  5],
	[10,  5,  6, 10,  8,  5, 10,  3,  8,  0,  8,  3,  2,  9,  1],
	[ 6, 10,  3,  6,  3,  5,  2,  9,  3,  9,  5,  3, -1, -1, -1],
	[ 2,  7,  6,  3,  7,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  7,  6,  2,  3,  7,  0,  1,  8, -1, -1, -1, -1, -1, -1],
	[ 7,  0, 11,  7,  6,  0,  6,  2,  0, -1, -1, -1, -1, -1, -1],
	[ 7,  6,  2,  7,  2,  8,  7,  8, 11,  8,  2,  1, -1, -1, -1],
	[ 7,  2,  3,  7,  6,  2,  5,  4,  8, -1, -1, -1, -1, -1, -1],
	[ 1,  4,  0,  1,  5,  4,  3,  7,  2,  7,  6,  2, -1, -1, -1],
	[ 5,  4,  8,  7,  6, 11,  6,  0, 11,  6,  2,  0, -1, -1, -1],
	[ 6,  2, 11,  6, 11,  7,  2,  1, 11,  4, 11,  5,  1,  5, 11],
	[11,  2,  3, 11,  4,  2,  4,  6,  2, -1, -1, -1, -1, -1, -1],
	[ 1,  8,  0,  2,  3,  4,  2,  4,  6,  4,  3, 11, -1, -1, -1],
	[ 2,  0,  6,  6,  0,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  8,  4,  1,  4,  2,  2,  4,  6, -1, -1, -1, -1, -1, -1],
	[ 3, 11,  8,  3,  8,  6,  3,  6,  2,  5,  6,  8, -1, -1, -1],
	[ 1,  5, 11,  1, 11,  0,  5,  6, 11,  3, 11,  2,  6,  2, 11],
	[ 8,  5,  6,  8,  6,  0,  0,  6,  2, -1, -1, -1, -1, -1, -1],
	[ 6,  2,  1,  5,  6,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 9,  7,  6,  9,  1,  7,  1,  3,  7, -1, -1, -1, -1, -1, -1],
	[ 0,  3,  7,  0,  7,  9,  0,  9,  8,  6,  9,  7, -1, -1, -1],
	[ 0, 11,  7,  1,  0,  7,  1,  7,  6,  1,  6,  9, -1, -1, -1],
	[ 7,  6,  9,  7,  9, 11, 11,  9,  8, -1, -1, -1, -1, -1, -1],
	[ 8,  5,  4,  9,  1,  6,  1,  7,  6,  1,  3,  7, -1, -1, -1],
	[ 4,  0,  9,  4,  9,  5,  0,  3,  9,  6,  9,  7,  3,  7,  9],
	[ 1,  6,  9,  1,  7,  6,  1,  0,  7, 11,  7,  0,  8,  5,  4],
	[ 5,  4, 11,  5, 11,  9,  7,  6, 11,  6,  9, 11, -1, -1, -1],
	[ 9,  1,  3,  9,  3,  4,  9,  4,  6,  4,  3, 11, -1, -1, -1],
	[ 4,  6,  3,  4,  3, 11,  6,  9,  3,  0,  3,  8,  9,  8,  3],
	[ 9,  1,  0,  9,  0,  6,  6,  0,  4, -1, -1, -1, -1, -1, -1],
	[ 4,  6,  9,  8,  4,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  3,  6,  1,  6,  9,  3, 11,  6,  5,  6,  8, 11,  8,  6],
	[ 3, 11,  0,  9,  5,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 8,  5,  6,  8,  6,  0,  9,  1,  6,  1,  0,  6, -1, -1, -1],
	[ 6,  9,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  5,  9,  7,  5, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5, 10,  7,  5,  9, 10,  1,  8,  0, -1, -1, -1, -1, -1, -1],
	[10,  5,  9, 10,  7,  5, 11,  3,  0, -1, -1, -1, -1, -1, -1],
	[10,  7,  9,  9,  7,  5, 11,  3,  1, 11,  1,  8, -1, -1, -1],
	[ 4, 10,  7,  4,  8, 10,  8,  9, 10, -1, -1, -1, -1, -1, -1],
	[ 7,  9, 10,  7,  0,  9,  7,  4,  0,  1,  9,  0, -1, -1, -1],
	[ 3,  0, 11, 10,  7,  8, 10,  8,  9,  8,  7,  4, -1, -1, -1],
	[ 3,  1,  4,  3,  4, 11,  1,  9,  4,  7,  4, 10,  9, 10,  4],
	[ 5, 11,  4,  5,  9, 11,  9, 10, 11, -1, -1, -1, -1, -1, -1],
	[ 0,  1,  8, 11,  4,  9, 11,  9, 10,  9,  4,  5, -1, -1, -1],
	[ 3,  0,  4,  3,  4,  9,  3,  9, 10,  9,  4,  5, -1, -1, -1],
	[ 9, 10,  4,  9,  4,  5, 10,  3,  4,  8,  4,  1,  3,  1,  4],
	[10, 11,  9, 11,  8,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 0,  1,  9,  0,  9, 11, 11,  9, 10, -1, -1, -1, -1, -1, -1],
	[ 3,  0,  8,  3,  8, 10, 10,  8,  9, -1, -1, -1, -1, -1, -1],
	[ 3,  1,  9, 10,  3,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[10,  1,  2, 10,  7,  1,  7,  5,  1, -1, -1, -1, -1, -1, -1],
	[10,  7,  5, 10,  5,  0, 10,  0,  2,  0,  5,  8, -1, -1, -1],
	[11,  3,  0, 10,  7,  2,  7,  1,  2,  7,  5,  1, -1, -1, -1],
	[ 7,  5,  2,  7,  2, 10,  5,  8,  2,  3,  2, 11,  8, 11,  2],
	[ 1,  4,  8,  2,  4,  1,  2,  7,  4,  2, 10,  7, -1, -1, -1],
	[10,  7,  4, 10,  4,  2,  2,  4,  0, -1, -1, -1, -1, -1, -1],
	[ 2,  8,  1,  2,  4,  8,  2, 10,  4,  7,  4, 10,  3,  0, 11],
	[11,  3,  2, 11,  2,  4, 10,  7,  2,  7,  4,  2, -1, -1, -1],
	[ 2,  5,  1,  2, 11,  5,  2, 10, 11,  4,  5, 11, -1, -1, -1],
	[ 0,  2,  5,  0,  5,  8,  2, 10,  5,  4,  5, 11, 10, 11,  5],
	[ 0,  4, 10,  0, 10,  3,  4,  5, 10,  2, 10,  1,  5,  1, 10],
	[ 2, 10,  3,  8,  4,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  2, 10,  1, 10,  8,  8, 10, 11, -1, -1, -1, -1, -1, -1],
	[10, 11,  0,  2, 10,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  2, 10,  1, 10,  8,  3,  0, 10,  0,  8, 10, -1, -1, -1],
	[ 2, 10,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  5,  9,  2,  3,  5,  3,  7,  5, -1, -1, -1, -1, -1, -1],
	[ 0,  1,  8,  2,  3,  9,  3,  5,  9,  3,  7,  5, -1, -1, -1],
	[ 9,  2,  0,  9,  0,  7,  9,  7,  5, 11,  7,  0, -1, -1, -1],
	[ 8, 11,  2,  8,  2,  1, 11,  7,  2,  9,  2,  5,  7,  5,  2],
	[ 2,  8,  9,  2,  7,  8,  2,  3,  7,  7,  4,  8, -1, -1, -1],
	[ 3,  7,  9,  3,  9,  2,  7,  4,  9,  1,  9,  0,  4,  0,  9],
	[ 8,  9,  7,  8,  7,  4,  9,  2,  7, 11,  7,  0,  2,  0,  7],
	[ 2,  1,  9,  7,  4, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  4,  5,  3, 11,  5,  3,  5,  9,  3,  9,  2, -1, -1, -1],
	[ 3,  9,  2,  3,  5,  9,  3, 11,  5,  4,  5, 11,  0,  1,  8],
	[ 5,  9,  2,  5,  2,  4,  4,  2,  0, -1, -1, -1, -1, -1, -1],
	[ 5,  9,  2,  5,  2,  4,  1,  8,  2,  8,  4,  2, -1, -1, -1],
	[ 2,  3, 11,  2, 11,  9,  9, 11,  8, -1, -1, -1, -1, -1, -1],
	[ 0,  1,  9,  0,  9, 11,  2,  3,  9,  3, 11,  9, -1, -1, -1],
	[ 2,  0,  8,  9,  2,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 2,  1,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3,  7,  1,  1,  7,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 8,  0,  3,  8,  3,  5,  5,  3,  7, -1, -1, -1, -1, -1, -1],
	[ 0, 11,  7,  0,  7,  1,  1,  7,  5, -1, -1, -1, -1, -1, -1],
	[ 8, 11,  7,  5,  8,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 4,  8,  1,  4,  1,  7,  7,  1,  3, -1, -1, -1, -1, -1, -1],
	[ 3,  7,  4,  0,  3,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 4,  8,  1,  4,  1,  7,  0, 11,  1, 11,  7,  1, -1, -1, -1],
	[11,  7,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[11,  4,  5, 11,  5,  3,  3,  5,  1, -1, -1, -1, -1, -1, -1],
	[ 8,  0,  3,  8,  3,  5, 11,  4,  3,  4,  5,  3, -1, -1, -1],
	[ 5,  1,  0,  4,  5,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 5,  8,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  3, 11,  8,  1, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 3, 11,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[ 1,  0,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
	[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
];

pub fn vert_interp(point1: (Vec3, f32), point2: (Vec3, f32)) -> Vec3
{
    if point1.1.abs() < 0.00001 { return point1.0; }
    if point2.1.abs() < 0.00001 { return point2.0; }
    if (point1.1 - point2.1).abs() < 0.00001 { return point1.0; }

    let t = (-point1.1 / (point2.1 - point1.1)).clamp(0.0,1.0);
    return Lerp::lerp(point1.0, point2.0, t);
}