public class USQUE {
// for now, this class is mostly a shell for Java Native Interface calls to the
// filter
	
	static {
		System.loadLibrary("USQUE");
	}

	public static native String filterStep(String json_params);

}
