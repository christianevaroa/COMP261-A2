
public class tester {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		double est = 1.7372939622694634;
		System.out.println("est: "+est);
		System.out.println("est 0:"+(est-(est*0.05)));
		System.out.println("est 1:"+(est-(est*0.20)));
		System.out.println("est 2:"+(est-(est*0.40)));
		System.out.println("est 3:"+(est-(est*0.60)));
		System.out.println("est 4:"+(est-(est*0.80)));
		System.out.println("est 5:"+(est-(est*0.99)));

	}

}
