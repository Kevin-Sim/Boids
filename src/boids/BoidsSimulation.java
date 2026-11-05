package boids;


// BoidsSimulation.java
// Complete single-file Boids simulation with QuadTree and sliders.
import javax.swing.*;
import javax.swing.Timer;
import javax.swing.event.*;
import java.awt.*;
import java.util.*;
import java.util.List;

public class BoidsSimulation extends JPanel {
	public static final int WIDTH = 1500, HEIGHT = 1200;
	public static final int NUMBER_OF_BOIDS = 2000;
	public static final int BOID_SIZE = 3;

	List<Boid> boids = new ArrayList<>();
	double separationWeight = 1.8, alignmentWeight = 1.0, cohesionWeight = 1.0;
	double neighbourRadius = 60, separationRadius = 25, maxSpeed = 4.0, maxForce = 0.08;

	public BoidsSimulation() {
		setPreferredSize(new Dimension(WIDTH, HEIGHT));
		setBackground(Color.BLACK);
		Random r = new Random();
		for (int i = 0; i < NUMBER_OF_BOIDS; i++)
			boids.add(new Boid(r.nextDouble() * WIDTH, r.nextDouble() * HEIGHT));
		new Timer(16, e -> {
			QuadTree qt = new QuadTree(new Rect(WIDTH / 2.0, HEIGHT / 2.0, WIDTH / 2.0, HEIGHT / 2.0), 10);
			for (Boid b : boids)
				qt.insert(b);
			for (Boid b : boids) {
				List<Boid> n = new ArrayList<>();
				qt.query(new Rect(b.x, b.y, neighbourRadius, neighbourRadius), n);
				b.flock(n, separationWeight, alignmentWeight, cohesionWeight, neighbourRadius, separationRadius,
						maxSpeed, maxForce);
				b.update();
				b.wrap(WIDTH, HEIGHT);
			}
			repaint();
		}).start();
	}

	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		for (Boid b : boids)
			b.draw((Graphics2D) g);
	}

	public static void main(String[] a) {
		SwingUtilities.invokeLater(() -> {
			JFrame f = new JFrame("Boids QuadTree");
			BoidsSimulation s = new BoidsSimulation();
			JPanel p = new JPanel(new GridLayout(7, 2));
			JLabel[] l = { new JLabel(), new JLabel(), new JLabel(), new JLabel(), new JLabel(), new JLabel(),
					new JLabel() };
			JSlider sw = new JSlider(0, 500, 180), aw = new JSlider(0, 500, 100), cw = new JSlider(0, 500, 100),
					nr = new JSlider(10, 200, 60), sr = new JSlider(5, 100, 25), ms = new JSlider(10, 100, 40),
					mf = new JSlider(1, 100, 8);
			ChangeListener cl = e -> {
				s.separationWeight = sw.getValue() / 100.0;
				s.alignmentWeight = aw.getValue() / 100.0;
				s.cohesionWeight = cw.getValue() / 100.0;
				s.neighbourRadius = nr.getValue();
				s.separationRadius = sr.getValue();
				s.maxSpeed = ms.getValue() / 10.0;
				s.maxForce = mf.getValue() / 100.0;
				l[0].setText("Separation " + String.format("%.2f", s.separationWeight));
				l[1].setText("Alignment " + String.format("%.2f", s.alignmentWeight));
				l[2].setText("Cohesion " + String.format("%.2f", s.cohesionWeight));
				l[3].setText("Neighbour " + (int) s.neighbourRadius);
				l[4].setText("Sep Radius " + (int) s.separationRadius);
				l[5].setText("Max Speed " + String.format("%.2f", s.maxSpeed));
				l[6].setText("Max Force " + String.format("%.2f", s.maxForce));
			};
			for (JSlider sl : new JSlider[] { sw, aw, cw, nr, sr, ms, mf })
				sl.addChangeListener(cl);
			cl.stateChanged(null);
			JSlider[] arr = { sw, aw, cw, nr, sr, ms, mf };
			for (int i = 0; i < 7; i++) {
				p.add(l[i]);
				p.add(arr[i]);
			}
			f.add(s);
			f.add(p, BorderLayout.SOUTH);
			f.pack();
			f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			f.setVisible(true);
		});
	}
}

class Boid {
	double x, y, vx, vy;
	Random r = new Random();

	Boid(double x, double y) {
		this.x = x;
		this.y = y;
		vx = r.nextDouble() * 4 - 2;
		vy = r.nextDouble() * 4 - 2;
	}

	void flock(List<Boid> b, double sw, double aw, double cw, double nr, double sr, double ms, double mf) {
		Vector2 sep = separate(b, sr, ms, mf), ali = align(b, nr, ms, mf), coh = cohesion(b, nr, ms, mf);
		sep.mult(sw);
		ali.mult(aw);
		coh.mult(cw);
		vx += sep.x + ali.x + coh.x;
		vy += sep.y + ali.y + coh.y;
		limit(ms);
	}

	Vector2 separate(List<Boid> bs, double r, double ms, double mf) {
		Vector2 st = new Vector2();
		int c = 0;
		for (Boid o : bs) {
			if (o == this)
				continue;
			double d = dist(o);
			if (d < r && d > 0) {
				Vector2 diff = new Vector2(x - o.x, y - o.y);
				diff.normalize();
				diff.div(d);
				st.add(diff);
				c++;
			}
		}
		if (c > 0)
			st.div(c);
		if (st.mag() > 0) {
			st.normalize();
			st.mult(ms);
			st.sub(new Vector2(vx, vy));
			st.limit(mf);
		}
		return st;
	}

	Vector2 align(List<Boid> bs, double r, double ms, double mf) {
		Vector2 a = new Vector2();
		int c = 0;
		for (Boid o : bs) {
			if (o != this && dist(o) < r) {
				a.add(new Vector2(o.vx, o.vy));
				c++;
			}
		}
		if (c == 0)
			return new Vector2();
		a.div(c);
		a.normalize();
		a.mult(ms);
		a.sub(new Vector2(vx, vy));
		a.limit(mf);
		return a;
	}

	Vector2 cohesion(List<Boid> bs, double r, double ms, double mf) {
		Vector2 ctr = new Vector2();
		int c = 0;
		for (Boid o : bs) {
			if (o != this && dist(o) < r) {
				ctr.add(new Vector2(o.x, o.y));
				c++;
			}
		}
		if (c == 0)
			return new Vector2();
		ctr.div(c);
		return seek(ctr, ms, mf);
	}

	Vector2 seek(Vector2 t, double ms, double mf) {
		Vector2 d = new Vector2(t.x - x, t.y - y);
		d.normalize();
		d.mult(ms);
		d.sub(new Vector2(vx, vy));
		d.limit(mf);
		return d;
	}

	double dist(Boid o) {
		double dx = x - o.x, dy = y - o.y;
		return Math.sqrt(dx * dx + dy * dy);
	}

	void update() {
		x += vx;
		y += vy;
	}

	void wrap(int w, int h) {
		if (x < 0)
			x = w;
		if (x > w)
			x = 0;
		if (y < 0)
			y = h;
		if (y > h)
			y = 0;
	}

	void limit(double ms) {
		double s = Math.sqrt(vx * vx + vy * vy);
		if (s > ms) {
			vx = vx / s * ms;
			vy = vy / s * ms;
		}
	}

	void draw(Graphics2D g) {
		double a = Math.atan2(vy, vx);
		int sz = BoidsSimulation.BOID_SIZE;
		Polygon p = new Polygon();
		p.addPoint((int) (x + Math.cos(a) * sz), (int) (y + Math.sin(a) * sz));
		p.addPoint((int) (x + Math.cos(a + 2.5) * sz), (int) (y + Math.sin(a + 2.5) * sz));
		p.addPoint((int) (x + Math.cos(a - 2.5) * sz), (int) (y + Math.sin(a - 2.5) * sz));
		g.setColor(Color.CYAN);
		g.fillPolygon(p);
	}
}

class Vector2 {
	double x, y;

	Vector2() {
		this(0, 0);
	}

	Vector2(double x, double y) {
		this.x = x;
		this.y = y;
	}

	void add(Vector2 v) {
		x += v.x;
		y += v.y;
	}

	void sub(Vector2 v) {
		x -= v.x;
		y -= v.y;
	}

	void mult(double n) {
		x *= n;
		y *= n;
	}

	void div(double n) {
		if (n != 0) {
			x /= n;
			y /= n;
		}
	}

	double mag() {
		return Math.sqrt(x * x + y * y);
	}

	void normalize() {
		double m = mag();
		if (m > 0)
			div(m);
	}

	void limit(double m) {
		if (mag() > m) {
			normalize();
			mult(m);
		}
	}
}

class Rect {
	double x, y, w, h;

	Rect(double x, double y, double w, double h) {
		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
	}

	boolean contains(Boid b) {
		return b.x >= x - w && b.x <= x + w && b.y >= y - h && b.y <= y + h;
	}

	boolean intersects(Rect o) {
		return !(o.x - o.w > x + w || o.x + o.w < x - w || o.y - o.h > y + h || o.y + o.h < y - h);
	}
}

class QuadTree {
	Rect b;
	int cap;
	java.util.List<Boid> pts = new ArrayList<>();
	boolean div = false;
	QuadTree ne, nw, se, sw;

	QuadTree(Rect b, int c) {
		this.b = b;
		cap = c;
	}

	boolean insert(Boid bo) {
		if (!b.contains(bo))
			return false;
		if (pts.size() < cap) {
			pts.add(bo);
			return true;
		}
		if (!div)
			sub();
		return ne.insert(bo) || nw.insert(bo) || se.insert(bo) || sw.insert(bo);
	}

	void sub() {
		double x = b.x, y = b.y, w = b.w / 2, h = b.h / 2;
		ne = new QuadTree(new Rect(x + w, y - h, w, h), cap);
		nw = new QuadTree(new Rect(x - w, y - h, w, h), cap);
		se = new QuadTree(new Rect(x + w, y + h, w, h), cap);
		sw = new QuadTree(new Rect(x - w, y + h, w, h), cap);
		div = true;
	}

	void query(Rect r, List<Boid> f) {
		if (!b.intersects(r))
			return;
		for (Boid p : pts)
			if (r.contains(p))
				f.add(p);
		if (!div)
			return;
		ne.query(r, f);
		nw.query(r, f);
		se.query(r, f);
		sw.query(r, f);
	}
}
